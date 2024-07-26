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
#include <deque>
#include <algorithm>
#include <unordered_set>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;


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

class GoalPointsGenerator : public rclcpp::Node
{
public: 
    GoalPointsGenerator() : Node("goal_points_generator"), x_c(0), y_c(0), yaw_c(0),
                            number_of_safest_headings(0),
                            tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
                            tf_listener_(*tf_buffer_),
                            goal_reached_(false),
                            first_goal_received(false),
                            exploration_called(false),
                            rotating(false),
                            received_microphone_heading(false),
                            last_goal_reached_time(this->now())
    {
        // Create a subscriber to the "headings" topic from the microphone
        headings_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "/microphone/headings", 1,
            std::bind(&GoalPointsGenerator::microphoneGoalCallback, this, std::placeholders::_1));
        
        safestAvailable_headings_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/available_headings/safest", 1,
            std::bind(&GoalPointsGenerator::safestAv_Headings_Callback, this, std::placeholders::_1));

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/icp_odom", 10,
            std::bind(&GoalPointsGenerator::odomCallback, this, std::placeholders::_1));
        
        AllAvailable_headings_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/available_headings/all", 10,
            std::bind(&GoalPointsGenerator::AllAvailableHeadingsCallback, this, std::placeholders::_1));
        
        goal_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        
        goal_state_publisher = this->create_publisher<std_msgs::msg::Bool>(
            "/goal_state", 10);

        explore_command_publisher = this->create_publisher<std_msgs::msg::Bool>(
            "/explore_command", 1);

        navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&GoalPointsGenerator::explore, this));
        // Initialize the waypoints set
        visited_waypoints_ = std::make_shared<std::unordered_set<Waypoint>>();
        x_global_frame = 0.0; y_global_frame = 0.0; yaw_global_frame = 0.0;
    
    }

private:

    // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        
    //     geometry_msgs::msg::PoseStamped pose_in, pose_out;
    //     pose_in.pose = msg->pose.pose;
    //     pose_in.header = msg->header;
    //     //Transform from "odom" frame to velodyne frame
    //     try {
    //         tf_buffer_->transform(pose_in, pose_out, "velodyne", tf2::durationFromSec(1.0));
    //     } catch (tf2::TransformException &ex) {
    //         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
    //         return;
    //     }

    //     //Extract the positions
    //     x_c = pose_out.pose.position.x;
    //     y_c = pose_out.pose.position.y;

    //     // Extract Yaw angle
    //     double quat_x = pose_out.pose.orientation.x;
    //     double quat_y = pose_out.pose.orientation.y;
    //     double quat_z = pose_out.pose.orientation.z;
    //     double quat_w = pose_out.pose.orientation.w;

    //     double siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y);
    //     double cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
    //     yaw_c = atan2(siny_cosp, cosy_cosp);

    //     //Publish the goal state as a flag for publishing available headings
    //     if (goal_reached_ || !first_goal_received) {
    //         std_msgs::msg::Bool state_msg;
	//     rclcpp::sleep_for(std::chrono::milliseconds(10000));
    //         state_msg.data = true;
    //         goal_state_publisher->publish(state_msg);
    //     } else {
    //         std_msgs::msg::Bool state_msg;
    //         state_msg.data = false;
    //         goal_state_publisher->publish(state_msg);
    //     }
    // }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        geometry_msgs::msg::PoseStamped pose_in1, pose_out1;
        pose_in1.pose = msg->pose.pose;
        pose_in1.header = msg->header;
        //Transform from "odom" frame to velodyne frame
        try {
            tf_buffer_->transform(pose_in1, pose_out1, "map", tf2::durationFromSec(2.0));
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
            tf_buffer_->transform(pose_in, pose_out, "velodyne", tf2::durationFromSec(2.0));
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
	        rclcpp::sleep_for(std::chrono::milliseconds(5000));
            state_msg.data = true;
            goal_state_publisher->publish(state_msg);
        } else {
            std_msgs::msg::Bool state_msg;
            state_msg.data = false;
            goal_state_publisher->publish(state_msg);
        }

    }


    void safestAv_Headings_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        number_of_safest_headings = msg->data.size();   
    }

    void AllAvailableHeadingsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        all_available_headings_ = msg->data;
    }

    void microphoneGoalCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {   
        if (goal_reached_ || !first_goal_received) {
            // Heading is an angle between -pi to pi (in radians)
            received_microphone_heading = true;
            double heading = static_cast<double>(msg->data);
            moveTowardsHeading(heading);
            first_goal_received = true;
            received_microphone_heading = false; 
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
                last_goal_reached_time = this->now();
		if (rotating) {
                    rotating = false;
                    Reset_Exploration_Direction(99999);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
		goal_reached_ = true;
                last_goal_reached_time = this->now();
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
		goal_reached_ = true;
                last_goal_reached_time = this->now();
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }

    // void moveTowardsHeading(double heading)
    // {
    //     double distance = 1.0;  // Define the distance for travel

    //     // Calculate the goal positions based on the current position and heading
    //     double x_goal = x_c + distance * cos(heading);
    //     double y_goal = y_c + distance * sin(heading);

    //     // Convert heading angle to quaternion
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, heading);

    //     geometry_msgs::msg::PoseStamped goal_pose;
    //     goal_pose.header.stamp = this->now();
    //     goal_pose.header.frame_id = "velodyne";  // Initial frame

    //     goal_pose.pose.position.x = x_goal;
    //     goal_pose.pose.position.y = y_goal;
    //     goal_pose.pose.position.z = 0.0;
    //     goal_pose.pose.orientation = tf2::toMsg(q);

    //     geometry_msgs::msg::PoseStamped transformed_goal_pose;

    //     // Transform the goal_pose to the "map" frame
    //     try {
    //         // Use the same timestamp for the transform lookup
    //         tf_buffer_->transform(goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(1.0));

    //         // Publish the transformed goal_pose
    //         sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal_pose));
    //     } catch (tf2::TransformException& ex) {
    //         RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
    //     }
    // }

    void explore()
    {
        if (goal_reached_ || !first_goal_received)
        {
            //Calculate the approperiate time for waiting for the microphone
            //The microphone takes 5 seconds to listen for each heading
            double time_to_wait = (number_of_safest_headings * 5) + 5;
            // Check if more than 10 seconds have passed since the last heading message

            if ((this->now() - last_goal_reached_time).seconds() > 0)
            {
                RCLCPP_INFO(this->get_logger(), "No heading message received for %f seconds.", time_to_wait);
                
                // Start the exploration process
                //A flag is used to ensure that the startExploration function won't start before the goal >1 is reached 
                exploration_called = true;
                RCLCPP_INFO(this->get_logger(), "Exploration algorithm is running until a heading is received from the microphone.");
                startExploration();
                
                // Reset the flag after starting the exploration process
                goal_reached_ = false;
                first_goal_received = true;
            }
        }
    }

    void startExploration()
    {
        while(!received_microphone_heading && exploration_called) {
            
            const double half_plane_range = M_PI / 2;  // 90 degrees

            // First, consider the semi-plane centered around the robot's current forward orientation
            double forward_center = yaw_c;
            double backward_center = yaw_c + M_PI;

            if (backward_center > M_PI) {
                backward_center -= 2 * M_PI;
            }

            // Create the half-planes based on the robot's current position and orientation
            Point current_position(x_global_frame, y_global_frame);

            // Half-plane at yaw_c + 90 degrees
            // Half-planes are constructed by taking the area to the LEFT of the vector
            double angle_minus_90 = yaw_global_frame - M_PI/2;
            //Normalize the angle to be within pi to -pi
            if (angle_minus_90 > M_PI) {
                angle_minus_90 -= 2 * M_PI;
            } else if (angle_minus_90 < -M_PI) {
                angle_minus_90 += 2 * M_PI;
            }
            Point direction_minus_90(cos(angle_minus_90), sin(angle_minus_90));
            Point end_minus_90 = current_position + direction_minus_90;

            Halfplane hp_minus_90(current_position, end_minus_90);
            halfplanes.push_back(hp_minus_90);
            RCLCPP_INFO(this->get_logger(), "Current Semi-plane was created and added to the list.");
            RCLCPP_INFO(this->get_logger(), "Semi-plane is the area to the LEFT of the vector from (%Lf,%Lf) to (%Lf, %Lf).", 
            current_position.x, current_position.y, end_minus_90.x, end_minus_90.y);
            RCLCPP_INFO(this->get_logger(), "Current yaw of the robot is: %f.", yaw_global_frame);

            // Compute the intersection of half-planes
            std::vector<Point> intersection = hp_intersect(halfplanes);

            // In the startExploration function, before returning the intersection points
            std::ofstream intersection_file;
            intersection_file.open("src/diablo_nav2_simulation/diablo_nav2/src/intersection_points.txt");
            int point_num = 0;
            std::cout << "##################" << std::endl;
            std::cout << "The intersection consists of the points: " << std::endl;
            for (const auto& point : intersection) {
                point_num++;
                std::cout << "Point " << point_num << " : (" << point.x << "," << point.y << ")" << std::endl;
                intersection_file << point.x << " " << point.y << "\n";
            }
            intersection_file.close();
             /*
                For each point in the intersection (which is the convex polygon formed by the intersection of the semi-planes), check if the direction vector keeps the robot inside the intersection.
                This is done using the cross product of the direction vector and the vector from the current position to each point in the intersection. If the cross product is negative (considering a small epsilon value for precision), the point is to the right of the direction vector, indicating that the robot would move outside the intersection if it followed this heading.
                If for any point in the intersection, the cross product is negative, the heading is not valid (i.e., it would lead the robot outside the intersection), and we set the flag inside to false.
             */
            // Find the best heading within the intersection
            if (!intersection.empty()) {
                double best_heading;
                bool heading_found = false;
                bool atleast1_valid_heading = false;
                double min_angle_diff = std::numeric_limits<double>::max();

                for (const auto& heading : all_available_headings_) {
                    // double angle_diff = heading + yaw_global_frame;
                    // // Normalize the angle to be within pi to -pi
                    // if (angle_diff > M_PI) {
                    //     angle_diff -= 2 * M_PI;
                    // } else if (angle_diff < -M_PI) {
                    //     angle_diff += 2 * M_PI;
                    // }
                    // Point goal_position(
                    //     x_global_frame + cos(angle_diff),
                    //     y_global_frame + sin(angle_diff)
                    // );

                    geometry_msgs::msg::PoseStamped goal_pose = calculate_goal_position(heading);
                    Point goal_position(
                        goal_pose.pose.position.x,
                        goal_pose.pose.position.y
                    );

                    std::cout << "Goal point is (" << goal_position.x << "," << goal_position.y << "), for the heading: " << heading << std::endl;

                    if (point_in_polygon(goal_position, intersection)) {
                        atleast1_valid_heading = true;
                        // Check if this goal is near any visited waypoints
                        bool near_visited_waypoint = false;
                        for (const auto& wp : *visited_waypoints_) {
                            if (std::fabs(goal_position.x - wp.x) < 0.2 && 
                                std::fabs(goal_position.y - wp.y) < 0.2) {
                                std::cout << "Goal point is near a visisted point, skipping it.." << std::endl;
                                near_visited_waypoint = true;
                                break;
                            }
                        }

                        if (!near_visited_waypoint) {
                            best_heading = heading;
                            heading_found = true;
                            break;
                        }
                        // best_heading = heading;
                        // heading_found = true;
                        // break;
                    } 
                }

                if (!heading_found && atleast1_valid_heading) {
                    for (const auto& heading : all_available_headings_) {
                        // double angle_diff = heading + yaw_global_frame;
                        // // Normalize the angle to be within pi to -pi
                        // if (angle_diff > M_PI) {
                        //     angle_diff -= 2 * M_PI;
                        // } else if (angle_diff < -M_PI) {
                        //     angle_diff += 2 * M_PI;
                        // }
                        // Point goal_position(
                        //     x_global_frame + cos(angle_diff),
                        //     y_global_frame + sin(angle_diff)
                        // );

                        geometry_msgs::msg::PoseStamped goal_pose = calculate_goal_position(heading);
                        Point goal_position(
                            goal_pose.pose.position.x,
                            goal_pose.pose.position.y
                        );
                        //Calculate the goal yaw
                        double quat_x = goal_pose.pose.orientation.x;
                        double quat_y = goal_pose.pose.orientation.y;
                        double quat_z = goal_pose.pose.orientation.z;
                        double quat_w = goal_pose.pose.orientation.w;

                        double siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y);
                        double cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
                        double angle_diff = atan2(siny_cosp, cosy_cosp);
                        
                        std::cout << "Goal point is (" << goal_position.x << "," << goal_position.y << "), for the heading: " << heading << std::endl;

                        if (point_in_polygon(goal_position, intersection)) {

                            // Find the heading that results in a different yaw degree
                            for (const auto& wp : *visited_waypoints_) {
                                double diff_yaw = atan2(sin(angle_diff - wp.yaw), cos(angle_diff - wp.yaw));
                                double opposite_yaw = atan2(sin(angle_diff - (wp.yaw + M_PI)), cos(angle_diff - (wp.yaw + M_PI)));

                                if ((std::fabs(diff_yaw) > (20 * M_PI / 180) || std::fabs(opposite_yaw) > (20 * M_PI / 180)) &&
                                    std::fabs(goal_position.x - wp.x) < 0.2 && 
                                    std::fabs(goal_position.y - wp.y) < 0.2) {
                                    std::cout << "Goal point is near a visisted point, but with different yaw degree. Goal Selected." << std::endl;
                                    best_heading = heading;
                                    heading_found = true;
                                    break;
                                }
                            }
                            // best_heading = heading;
                            // heading_found = true;
                            // break;
                        }
                        if (heading_found) break;
                    }
                }

                if (heading_found) {
                    moveTowardsHeading(best_heading);
                    RCLCPP_INFO(this->get_logger(), "The heading %f was found at the intersection of ALL semi-planes.", best_heading);
                    return;
                } else {
                    RCLCPP_WARN(this->get_logger(), "No available headings found in the intersection of the semi-planes.");
                }
            } 

            //Find the heading that is available and will cause the visit of unvisited cell
            RCLCPP_WARN(this->get_logger(), "Finding the new exploration direction.");
            double exploration_direction;
            bool exploration_dir_found = false;
            for (const auto& heading : all_available_headings_) {
                // double angle_diff = heading + yaw_global_frame;
                // // Normalize the angle to be within pi to -pi
                // if (angle_diff > M_PI) {
                //     angle_diff -= 2 * M_PI;
                // } else if (angle_diff < -M_PI) {
                //     angle_diff += 2 * M_PI;
                // }
                // Point goal_position(
                //     x_global_frame + cos(angle_diff),
                //     y_global_frame + sin(angle_diff)
                // );

                geometry_msgs::msg::PoseStamped goal_pose = calculate_goal_position(heading);
                Point goal_position(
                    goal_pose.pose.position.x,
                    goal_pose.pose.position.y
                );

                // Check if this goal is near any visited waypoints
                bool near_visited_waypoint = false;
                for (const auto& wp : *visited_waypoints_) {
                    if (std::fabs(goal_position.x - wp.x) < 0.2 && 
                        std::fabs(goal_position.y - wp.y) < 0.2) {
                        near_visited_waypoint = true;
                        break;
                    }
                }

                if (!near_visited_waypoint) {
                    exploration_direction = heading;
                    exploration_dir_found = true;
                    break;
                } 

            }
            
            //Perform the rotation and reset the explored planes
            if (exploration_dir_found) {
                RCLCPP_WARN(this->get_logger(), "The new exploration direction is: %f", exploration_direction);
                rotating = true;
                Reset_Exploration_Direction(exploration_direction);
            } else {
                //Make it rotate 180 degrees, regardless 
                RCLCPP_WARN(this->get_logger(), "No new exploration direction found. Rotating 180 degrees");
                rotating = true;
                Reset_Exploration_Direction(M_PI);
            }
        
            return;
        }
    }

    void Reset_Exploration_Direction(double direction)
    {
        if (rotating) {
            RCLCPP_INFO(this->get_logger(), "Rotating towards %f", direction);
            rotateToHeading(direction);
        } else {
            // Reset the flag 
            goal_reached_ = false;
            // Clear all the half-planes from the vector
            halfplanes.clear();
            // Restart the exploration process (will add the terminator later)
            exploration_called = true;
            startExploration();
        }
    }

    void rotateToHeading(double heading)
    {
        //new orientation is heading
        double new_orientation =  heading;
        std::cout << "New orientation is: " << new_orientation << std::endl;

        if (new_orientation > M_PI) {
            new_orientation -= 2 * M_PI;
        } else if (new_orientation < -M_PI) {
            new_orientation += 2 * M_PI;
        }
        // Create the goal pose for the initial position
        geometry_msgs::msg::PoseStamped rotate_goal_pose;
        rotate_goal_pose.header.stamp = this->now();
        rotate_goal_pose.header.frame_id = "velodyne";  

        //keep the current position
        rotate_goal_pose.pose.position.x = x_c;
        rotate_goal_pose.pose.position.y = y_c;
        rotate_goal_pose.pose.position.z = 0.0;

        // Set the initial orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, new_orientation);
        rotate_goal_pose.pose.orientation = tf2::toMsg(q);

        geometry_msgs::msg::PoseStamped transformed_goal_pose;
        // Transform the goal_pose to the "map" frame
        try {
            // Use the same timestamp for the transform lookup
            tf_buffer_->transform(rotate_goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(2.0));

            // Publish the transformed goal_pose
            sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal_pose));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
        }

        // Set a flag to indicate that the robot is rotating
        exploration_called = false;
        rotating = true;
    }


    void moveTowardsHeading(double heading)
    {
        //Stop Exploration untill the goal is reached
        exploration_called = false;
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
            tf_buffer_->transform(goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(2.0));

            // Publish the transformed goal_pose
            sendGoal(std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal_pose));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
        }
    }

    //Helper function that takes the heading and find the goal position in the map frame
    geometry_msgs::msg::PoseStamped calculate_goal_position(double heading) {

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
            tf_buffer_->transform(goal_pose, transformed_goal_pose, "map", tf2::durationFromSec(2.0));

            // Publish the transformed goal_pose
            return transformed_goal_pose;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform goal_pose: %s", ex.what());
        }
    }

    // Algorithm to find the polygon created by the intersection of half-planes
    std::vector<Point> hp_intersect(std::vector<Halfplane>& H) { 

        Point box[4] = {  // Bounding box in CCW order
            Point(inf, inf), 
            Point(-inf, inf), 
            Point(-inf, -inf), 
            Point(inf, -inf) 
        };

        for(int i = 0; i<4; i++) { // Add bounding box half-planes.
            Halfplane aux(box[i], box[(i+1) % 4]);
            H.push_back(aux);
        }

        // Sort by angle and start algorithm
        std::sort(H.begin(), H.end());
        std::deque<Halfplane> dq;
        int len = 0;
        for(int i = 0; i < int(H.size()); i++) {

            // Remove from the back of the deque while last half-plane is redundant
            while (len > 1 && H[i].out(inter(dq[len-1], dq[len-2]))) {
                dq.pop_back();
                --len;
            }

            // Remove from the front of the deque while first half-plane is redundant
            while (len > 1 && H[i].out(inter(dq[0], dq[1]))) {
                dq.pop_front();
                --len;
            }

            // Special case check: Parallel half-planes
            if (len > 0 && fabsl(cross(H[i].pq, dq[len-1].pq)) < eps) {
                // Opposite parallel half-planes that ended up checked against each other.
                if (dot(H[i].pq, dq[len-1].pq) < 0.0)
                    return std::vector<Point>();

                // Same direction half-plane: keep only the leftmost half-plane.
                if (H[i].out(dq[len-1].p)) {
                    dq.pop_back();
                    --len;
                }
                else continue;
            }

            // Add new half-plane
            dq.push_back(H[i]);
            ++len;
        }

        // Final cleanup: Check half-planes at the front against the back and vice-versa
        while (len > 2 && dq[0].out(inter(dq[len-1], dq[len-2]))) {
            dq.pop_back();
            --len;
        }

        while (len > 2 && dq[len-1].out(inter(dq[0], dq[1]))) {
            dq.pop_front();
            --len;
        }

        // Report empty intersection if necessary
        if (len < 3) return std::vector<Point>();

        // Reconstruct the convex polygon from the remaining half-planes.
        std::vector<Point> ret(len);
        for(int i = 0; i+1 < len; i++) {
            ret[i] = inter(dq[i], dq[i+1]);
        }
        ret.back() = inter(dq[len-1], dq[0]);
        return ret;
    }

    bool isWithinSemiPlane(double heading, double center_angle, double range) {
        double lower_bound = center_angle - range;
        double upper_bound = center_angle + range;

        if (lower_bound < -M_PI) lower_bound += 2 * M_PI;
        if (upper_bound > M_PI) upper_bound -= 2 * M_PI;

        if (lower_bound < upper_bound) {
            return heading >= lower_bound && heading <= upper_bound;
        } else {
            return heading >= lower_bound || heading <= upper_bound;
        }
    }

    // Checking if a point is inside a polygon
    bool point_in_polygon(Point point, const std::vector<Point>& polygon) {
        int num_vertices = polygon.size();
        long double x = point.x, y = point.y;
        bool inside = false;

        Point p1 = polygon[0], p2;

        for (int i = 1; i <= num_vertices; i++) {
            p2 = polygon[i % num_vertices];

            if (y > std::min(p1.y, p2.y)) {
                if (y <= std::max(p1.y, p2.y)) {
                    if (x <= std::max(p1.x, p2.x)) {
                        if (p1.y != p2.y) {
                            long double x_intersection = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                            if (p1.x == p2.x || x <= x_intersection) {
                                inside = !inside;
                            }
                        }
                    }
                }
            }

            p1 = p2;
        }

        return inside;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr headings_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr safestAvailable_headings_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr AllAvailable_headings_subscriber_;    
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_state_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_command_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_reached_, first_goal_received, exploration_called, received_microphone_heading;
    bool first_plane_created, rotating;
    double x_c, y_c, yaw_c;
    double x_global_frame, y_global_frame, yaw_global_frame;
    int number_of_safest_headings;
    std::vector<double> all_available_headings_;

    rclcpp::Time last_goal_reached_time;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    // Declare the waypoints container and halfplanes container
    std::vector<Halfplane> halfplanes;
    std::shared_ptr<std::unordered_set<Waypoint>> visited_waypoints_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPointsGenerator>());
    rclcpp::shutdown();
    return 0;
}
