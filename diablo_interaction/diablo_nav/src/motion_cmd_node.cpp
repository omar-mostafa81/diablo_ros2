#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
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
class MotionCmdNode : public rclcpp::Node
{
public:
    MotionCmdNode()
    : Node("motion_cmd_node")
    {
        // Create a subscriber to the /diablo/cmd_vel topic
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/diablo/cmd_vel", 10,
            std::bind(&MotionCmdNode::cmdVelCallback, this, std::placeholders::_1));

        // Create a publisher to the diablo/MotionCmd topic
        motion_publisher_ = this->create_publisher<motion_msgs::msg::MotionCtrl>(
            "diablo/MotionCmd", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto linear_v = msg->linear.x;
        auto angular_v = msg->angular.z;

        // Create a MotionCtrl message
        motion_msgs::msg::MotionCtrl cmd;
        cmd.mode_mark = false;
        cmd.value.forward = linear_v;
        cmd.value.left = angular_v;
        cmd.value.up = 1;

        // Publish the MotionCtrl message
        motion_publisher_->publish(cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionCmdNode>());
    rclcpp::shutdown();
    return 0;
}
