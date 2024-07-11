#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "std_msgs/msg/float64.hpp"
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

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class GoalPointsGenerator : public rclcpp::Node
{
public: 
    GoalPointsGenerator() : Node("goal_points_generator"), x_c(0), y_c(0), yaw_c(0),
                            tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
                            tf_listener_(*tf_buffer_),
                            goal_reached_(false),
                            first_goal_received(false)
    {
        // Create a subscriber to the "headings" topic from the microphone
        headings_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "/microphone/headings", 10,
            std::bind(&GoalPointsGenerator::goalCallback, this, std::placeholders::_1));
        
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/icp_odom", 10,
            std::bind(&GoalPointsGenerator::odomCallback, this, std::placeholders::_1));

        goal_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        
        goal_state_publisher = this->create_publisher<std_msgs::msg::Bool>(
            "/goal_state", 10);

        navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    }

private:

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        
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

        //Publish the goal state as a flag for publishing available headings
        if (goal_reached_ || !first_goal_received) {
            std_msgs::msg::Bool state_msg;
            state_msg.data = true;
            goal_state_publisher->publish(state_msg);
        } else {
            std_msgs::msg::Bool state_msg;
            state_msg.data = false;
            goal_state_publisher->publish(state_msg);
        }
    }

    void goalCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {   
        if (goal_reached_ || !first_goal_received) {

            // Heading is an angle between -pi to pi (in radians)
            double heading = static_cast<double>(msg->data);
            moveTowardsHeading(heading);
            first_goal_received = true;
        } else {
            std::cout << "Recieved a new heading from the microphone, but the robot didn't reach to the goal yet. Ignoring it..." << std::endl;
        }

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
	goal_reached_ = false;
    }

    void goalResponseCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
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

    void moveTowardsHeading(double heading)
    {
        double distance = 1.0;  // Define the distance for travel

        // Calculate the goal positions based on the current position and heading
        double x_goal = x_c + distance * cos(heading);
        double y_goal = y_c + distance * sin(heading);

        // Convert heading angle to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, heading);

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = this->now();
        goal_pose.header.frame_id = "velodyne";  // Initial frame

        goal_pose.pose.position.x = x_goal;
        goal_pose.pose.position.y = y_goal;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation = tf2::toMsg(q);

        geometry_msgs::msg::PoseStamped transformed_goal_pose;

        // Transform the goal_pose to the "map" frame
        try {
            // Use the same timestamp for the transform lookup
            tf_buffer_->transform(goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(1.0));

            // Publish the transformed goal_pose
            sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal_pose));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr headings_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_state_publisher;
    bool goal_reached_, first_goal_received;
    double x_c, y_c, yaw_c;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPointsGenerator>());
    rclcpp::shutdown();
    return 0;
}
