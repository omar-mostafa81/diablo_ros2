#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <utility> 
#include <fstream>
#include <filesystem>
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"


// Redefine epsilon and infinity as necessary. Be mindful of precision errors.
const long double eps = 1e-9, inf = 1e9; 

// Basic point/vector struct.
struct Point { 

    long double x, y;
    explicit Point(long double x = 0, long double y = 0) : x(x), y(y) {}

    // Addition, substraction, multiply by constant, dot product, cross product.

    friend Point operator + (const Point& p, const Point& q) {
        return Point(p.x + q.x, p.y + q.y); 
    }

    friend Point operator - (const Point& p, const Point& q) { 
        return Point(p.x - q.x, p.y - q.y); 
    }

    friend Point operator * (const Point& p, const long double& k) { 
        return Point(p.x * k, p.y * k); 
    } 

    friend long double dot(const Point& p, const Point& q) {
        return p.x * q.x + p.y * q.y;
    }

    friend long double cross(const Point& p, const Point& q) { 
        return p.x * q.y - p.y * q.x; 
    }
};

// Basic half-plane struct.
struct Halfplane { 

    // 'p' is a passing point of the line and 'pq' is the direction vector of the line.
    Point p, pq; 
    long double angle;

    Halfplane() {}
    Halfplane(const Point& a, const Point& b) : p(a), pq(b - a) {
        angle = atan2l(pq.y, pq.x);    
    }

    // Check if point 'r' is outside this half-plane. 
    // Every half-plane allows the region to the LEFT of its line.
    bool out(const Point& r) { 
        return cross(pq, r - p) < -eps; 
    }

    // Comparator for sorting. 
    bool operator < (const Halfplane& e) const { 
        return angle < e.angle;
    } 

    // Intersection point of the lines of two half-planes. It is assumed they're never parallel.
    friend Point inter(const Halfplane& s, const Halfplane& t) {
        long double alpha = cross((t.p - s.p), t.pq) / cross(s.pq, t.pq);
        return s.p + (s.pq * alpha);
    }
};

// Define a struct for the waypoint and its yaw
struct Waypoint {
    double x;
    double y;
    double yaw;

    bool operator==(const Waypoint &other) const {
        return std::fabs(x - other.x) < 0.1 &&
               std::fabs(y - other.y) < 0.1 &&
               std::fabs(yaw - other.yaw) < 0.1;
    }
};

// Define a hash function for Waypoint
namespace std {
    template<>
    struct hash<Waypoint> {
        size_t operator()(const Waypoint &wp) const {
            return std::hash<double>()(wp.x) ^
                   std::hash<double>()(wp.y) ^
                   std::hash<double>()(wp.yaw);
        }
    };
}

class ExplorationNode: public rclcpp::Node
{
public:
    ExplorationNode(): Node("explore_unknown_space") {
        
        AllAvailable_headings_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/available_headings/all", 10,
            std::bind(&ExplorationNode::AllAvailableHeadingsCallback, this, std::placeholders::_1));

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/icp_odom", 10,
            std::bind(&ExplorationNode::odomCallback, this, std::placeholders::_1));
        
        goal_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        
        goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&ExplorationNode::onGoalPoseReceived, this, std::placeholders::_1));

        explore_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/explore_command", 1,
            std::bind(&ExplorationNode::exploreCmdCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ExplorationNode::startExploration, this));

        navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        //Initialize the variables 
        explore_cmd = false;
        goal_reached_ = false;


    }

private:

    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        geometry_msgs::msg::PoseStamped pose_in1, pose_out1;
        pose_in1.pose = msg->pose.pose;
        pose_in1.header = msg->header;
        //Transform from "odom" frame to map frame
        try {
            tf_buffer_->transform(pose_in1, pose_out1, "map", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
            return;
        }

        //Extract the positions
        x_global_frame = pose_out1.pose.position.x;
        y_global_frame = pose_out1.pose.position.y;

        // Extract Yaw angle
        double quat_x1 = pose_out1.pose.orientation.x;
        double quat_y1 = pose_out1.pose.orientation.y;
        double quat_z1 = pose_out1.pose.orientation.z;
        double quat_w1 = pose_out1.pose.orientation.w;

        double siny_cosp1 = 2.0 * (quat_w1 * quat_z1 + quat_x1 * quat_y1);
        double cosy_cosp1 = 1.0 - 2.0 * (quat_y1 * quat_y1 + quat_z1 * quat_z1);
        yaw_global_frame = atan2(siny_cosp1, cosy_cosp1);

        // Save the odometry data as a waypoint
        Waypoint current_waypoint = {x_global_frame, y_global_frame, yaw_global_frame};
        visited_waypoints_->insert(current_waypoint);

        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.pose = msg->pose.pose;
        pose_in.header = msg->header;

        //Transform from "odom" frame to velodyne frame
        try {
            tf_buffer_->transform(pose_in, pose_out, "velodyne", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
            return;
        }

        //Extract the positions
        x_c = pose_out.pose.position.x;
        y_c = pose_out.pose.position.y;

        // Extract Yaw angle
        double quat_x = pose_out.pose.orientation.x;
        double quat_y = pose_out.pose.orientation.y;
        double quat_z = pose_out.pose.orientation.z;
        double quat_w = pose_out.pose.orientation.w;

        double siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y);
        double cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
        yaw_c = atan2(siny_cosp, cosy_cosp);
    }

    void AllAvailableHeadingsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        all_available_headings_ = msg->data;
    }

    void exploreCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        explore_cmd = msg->data;
    }

    void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Goal pose received");
        sendGoal(msg);
    }


    void sendGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose)
    {
        if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = *goal_pose;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](auto future) { this->goalResponseCallback(future); };
        send_goal_options.feedback_callback =
            [this](auto, auto feedback) { this->feedbackCallback(feedback); };
        send_goal_options.result_callback =
            [this](auto result) { this->resultCallback(result); };

        navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalResponseCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            goal_reached_ = true;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // RCLCPP_INFO(this->get_logger(), "Current pose: [%.2f, %.2f]", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "Navigation time: %.2f seconds", feedback->navigation_time.sec + feedback->navigation_time.nanosec / 1e9);
        // RCLCPP_INFO(this->get_logger(), "Estimated time remaining: %.2f seconds", feedback->estimated_time_remaining.sec + feedback->estimated_time_remaining.nanosec / 1e9);
        // RCLCPP_INFO(this->get_logger(), "Number of recoveries: %d", feedback->number_of_recoveries);
        // RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f meters", feedback->distance_remaining);
    }

    void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was reached");
                goal_reached_ = true;
                if (rotating) {
                    rotating = false;
                    Reset_Exploration_Direction(99999);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                goal_reached_ = true;
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                goal_reached_ = true;
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr AllAvailable_headings_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_cmd_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> all_available_headings_;
    bool explore_cmd, goal_reached_, rotating;
    double x_c, y_c, yaw_c;
    double x_global_frame, y_global_frame, yaw_global_frame;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<Halfplane> halfplanes;
    // Declare the waypoints container
    std::shared_ptr<std::unordered_set<Waypoint>> visited_waypoints_;



};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorationNode>());
    rclcpp::shutdown();
    return 0;
}
