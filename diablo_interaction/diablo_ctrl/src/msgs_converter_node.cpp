#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
//#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;

class DiabloSquarePathNode : public rclcpp::Node
{
public:
    DiabloSquarePathNode() : Node("msgs_converter_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node Started!");

        //Create the subscribers for each data source
        velodyne_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10, std::bind(&DiabloSquarePathNode::velodyneCallback, this, std::placeholders::_1));

        camera_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diablo1/odom", 10, std::bind(&DiabloSquarePathNode::cameraCallback, this, std::placeholders::_1));

        diablo_Imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/diablo/sensor/Imu", 10, std::bind(&DiabloSquarePathNode::ImuCallback, this, std::placeholders::_1));    

        //Create publisher for each message 
        velodyne_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/velodyne/PoseWithCovariance", 10);

        camera_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/camera/TwistWithCovariance", 10);
        
        diablo_Imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/diablo/ImuWithCovariance", 10);
    }

private:

    void velodyneCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract pitch, yaw, and roll from the quaternion orientation
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        //Extract position readings
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        //Create the PoseWithCovariance msg 
        geometry_msgs::msg::PoseWithCovarianceStamped velodyne_msg;

        velodyne_msg.header.stamp = this->get_clock()->now();
        velodyne_msg.header.frame_id = "odom";

        velodyne_msg.pose.pose.position.x = x;
        velodyne_msg.pose.pose.position.y = y;
        velodyne_msg.pose.pose.position.z = z;

        velodyne_msg.pose.pose.orientation.x = qx;
        velodyne_msg.pose.pose.orientation.y = qy;
        velodyne_msg.pose.pose.orientation.z = qz;
        velodyne_msg.pose.pose.orientation.w = qw;

        //put it yourself 
        for (int i = 0; i < 36; i++) {
             if (i == 0 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) {
                velodyne_msg.pose.covariance[i] = 0.01;
            } else {
                velodyne_msg.pose.covariance[i] = 0.0;
            }
        }

        velodyne_publisher_->publish(velodyne_msg);
    }

    void cameraCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract pitch, yaw, and roll from the quaternion orientation
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;

        //Extract position readings
        double ax = msg->twist.twist.angular.x;
        double ay = msg->twist.twist.angular.y;
        double az = msg->twist.twist.angular.z;

        //Create the PoseWithCovariance msg 
        geometry_msgs::msg::TwistWithCovarianceStamped camera_msg;

        camera_msg.header.stamp = this->get_clock()->now();
        camera_msg.header.frame_id = "odom";

        camera_msg.twist.twist.linear.x = vx;
        camera_msg.twist.twist.linear.y = vy;
        camera_msg.twist.twist.linear.z = vz;

        camera_msg.twist.twist.angular.x = ax;
        camera_msg.twist.twist.angular.y = ay;
        camera_msg.twist.twist.angular.z = az;

        //take it from the camera
        for (int i = 0; i < 36; i++) {
            camera_msg.twist.covariance[i] = msg->twist.covariance[i]; 
            if (camera_msg.twist.covariance[i] > 1) {
            	return;
            }
        }
       
       camera_publisher_->publish(camera_msg);
    }

        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract pitch, yaw, and roll from the quaternion orientation
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        
        //extract the angular velocity
        double vx = msg->angular_velocity.x;
        double vy = msg->angular_velocity.y;
        double vz = msg->angular_velocity.z;

        //extract the linear acceleration
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        //Create the PoseWithCovariance msg 
        sensor_msgs::msg::Imu Imu_msg;

        Imu_msg.header.stamp = this->get_clock()->now();
        Imu_msg.header.frame_id = "base_link";

        Imu_msg.orientation.x = qx;
        Imu_msg.orientation.y = qy;
        Imu_msg.orientation.z = qz;
        Imu_msg.orientation.w = qw;

        Imu_msg.angular_velocity.x = vx;
        Imu_msg.angular_velocity.y = vy;
        Imu_msg.angular_velocity.z = vz;

        Imu_msg.linear_acceleration.x = ax;
        Imu_msg.linear_acceleration.y = ay;
        Imu_msg.linear_acceleration.z = az;
        //Specify the orientation covariance
        for (int i = 0; i < 9; i++) {
            if (i == 0 || i == 4 || i == 8) {
                Imu_msg.orientation_covariance[i] = 0.1;
            } else {
                Imu_msg.orientation_covariance[i] = 0.0;
            }
        }

        //specify the angular velocity covariance
        for (int i = 0; i < 9; i++) {
            if (i == 0 || i == 4 || i == 8) {
                Imu_msg.angular_velocity_covariance[i] = 0.01;
            } else {
                Imu_msg.angular_velocity_covariance[i] = 0.0;
            }
        }

        //specify the linear acceleration covariance
        for (int i = 0; i < 9; i++) {
            if (i == 0 || i == 4 || i == 8) {
                Imu_msg.linear_acceleration_covariance[i] = 0.01;
            } else {
                Imu_msg.linear_acceleration_covariance[i] = 0.0;
            }
        }

        diablo_Imu_publisher_->publish(Imu_msg);
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velodyne_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr diablo_Imu_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr velodyne_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr diablo_Imu_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr camera_publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloSquarePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
