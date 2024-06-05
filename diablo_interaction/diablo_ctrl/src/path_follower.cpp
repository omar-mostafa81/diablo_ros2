//vicon + lidar
//Author: Omar Mostafa, CAIR Research Intern, NYUAD
//Date: Spring 2024
//Description: path follower for the diablo robot to follow a pre determined path, depends on an acceleration controller 
// This version uses the VICON instead of the LIDAR

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <utility> 
#include <fstream>
#include <filesystem>
#include <sstream>

using namespace std;

class DiabloSquarePathNode : public rclcpp::Node
{
public:
    DiabloSquarePathNode() : Node("path_follower_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node Started!");

        // Read and update file index
        int file_index = readAndUpdateFileIndex();

        // Generate unique filename
        std::stringstream file_name_ss;
        file_name_ss << "/home/risc-diablo1/diablo_ws/src/diablo_ros2/diablo_interaction/diablo_ctrl/src/data_files/data_test_" << file_index << ".txt";
        file_path_ = file_name_ss.str();

        //diablo_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
          // "/odometry/filtered", 30, std::bind(&DiabloSquarePathNode::poseCallback, this, std::placeholders::_1));
        
        vicon_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/diablo1/pose", 100, std::bind(&DiabloSquarePathNode::viconCallback, this, std::placeholders::_1));

        motion_publisher_ = this->create_publisher<motion_msgs::msg::MotionCtrl>(
            "diablo/MotionCmd", 10);

        // Initialize control parameters
        Kv = 0.4;       // Forward gain factor
        Kw = 1.0;      // Rotation gain factor
        
        initial_x_vicon = 0.0;
        initial_y_vicon = 0.0;
	    initial_yaw = 0.0;
        initial_x = 0;
        initial_y = 0;
        V_x_vicon = 0;
        V_y_vicon = 0;
        yaw_vicon = 0.0;
        last_x_vicon = 0;
        last_y_vicon = 0;

        x_c = 0.0;
        y_c = 0.0;
        yaw_c = 0.0;
        V_x = 0;
        V_y = 0;
        last_omega = 0;
        num_points = 1000;
        last_x = 0;
        last_y = 0;
        last_v = 0;
        linear_v = 0;
        x_vicon = 0;
        y_vicon = 0;
        curv = 0;
        initialized = false;
        first_loop = true;

        Dv_y = 0; 
        Dv_x = 0;
        shortest_d = 0;

        firstMsg = true;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DiabloSquarePathNode::start, this));

        // Create a new file for each run
        std::ofstream outfile(file_path_);
        RCLCPP_INFO(this->get_logger(), "File Created: %s", file_path_.c_str());
        outfile << "path_vector_x, path_vector_y" << std::endl;
        outfile.close();
    }

private:
    int readAndUpdateFileIndex()
    {
        std::string index_file_path = "/home/risc-diablo1/diablo_ws/src/diablo_ros2/diablo_interaction/diablo_ctrl/src/data_files/file_index.txt";
        int file_index = 0;

        // Read the current index from the file
        std::ifstream infile(index_file_path);
        if (infile.is_open() && infile >> file_index)
        {
            infile.close();
        }

        // Increment and update the index for next run
        std::ofstream outfile(index_file_path);
        outfile << file_index + 1;
        outfile.close();

        return file_index;
    }


    // void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    // {
    //     //Assign the last_x,y
    //     last_x = x_c;
    //     last_y = y_c;

    //     // Extract Yaw angle
	//     double quat_x = msg->pose.pose.orientation.x;
    //     double quat_y = msg->pose.pose.orientation.y;
    //     double quat_z = msg->pose.pose.orientation.z;
    //     double quat_w = msg->pose.pose.orientation.w;

    //     double siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y);
    //     double cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
    //     double yaw = atan2(siny_cosp, cosy_cosp);

    //     //Extract position and velocity readings
    //     double x = msg->pose.pose.position.x;
    //     double y = msg->pose.pose.position.y;

    //     double beta = 0.6;
    //     x_c = beta * x_c + (1-beta) * x;
    //     y_c =  beta * y_c + (1-beta) * y;
        
    //     //x_c = x;
    //     //y_c = y;
    //     yaw_c = yaw;

    //     double raw_Vx = (x_c - last_x)/0.04545;
    //     double raw_Vy = (y_c - last_y)/0.04545;

    //     //Correction of the velocity to decrease the noise caused by the derivative calculation of the positions
    //     double alpha = 0.85;

    //     V_x = alpha * V_x + (1-alpha) * raw_Vx;
    //     V_y = alpha * V_y + (1-alpha) * raw_Vy;
    //     //V_x = msg->twist.twist.linear.x;
    //     //V_y = msg->twist.twist.linear.y;

    // }

    // void viconCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    // {
    //     //Assign the last_x,y
    //     last_x_vicon = x_vicon;
    //     last_y_vicon = y_vicon;

    //     // Extract Yaw angle
	//     double quat_x = msg->pose.orientation.x;
    //     double quat_y = msg->pose.orientation.y;
    //     double quat_z = msg->pose.orientation.z;
    //     double quat_w = msg->pose.orientation.w;

    //     double siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y);
    //     double cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
    //     double yaw = atan2(siny_cosp, cosy_cosp);

    //     //Extract position and velocity readings
    //     double x = msg->pose.position.x;
    //     double y = msg->pose.position.y;
	
    //     if (firstMsg) {
    //         initial_x_vicon = x;
    //         initial_y_vicon = y;
	    
	//         firstMsg = false;
    //     }

    //     x_vicon = x - initial_x_vicon;
    //     y_vicon = x - initial_y_vicon;
    //     yaw_vicon = yaw;
    //     // x_c = x - initial_x_vicon;
    //     // y_c = y - initial_y_vicon; 
    //     // x_vicon = x;
    //     // y_vicon = y;
    //     // yaw_c = yaw;

    //     // double raw_Vx = (x_vicon - last_x_vicon)/0.0334;
    //     // double raw_Vy = (y_vicon - last_y_vicon)/0.0334;

    //     double raw_Vx = (x_vicon - last_x_vicon)/0.0334;
    //     double raw_Vy = (y_vicon - last_y_vicon)/0.0334;

    //     //Correction of the velocity to decrease the noise caused by the derivative calculation of the positions
    //     double alpha = 0.6;

    //     V_x_vicon = alpha * V_x_vicon + (1-alpha) * raw_Vx;
    //     V_y_vicon = alpha * V_y_vicon + (1-alpha) * raw_Vy;

    // }

    void viconCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        //Assign the last_x,y
        last_x = x_vicon;
        last_y = y_vicon;

        // Extract Yaw angle
	    double quat_x = msg->pose.orientation.x;
        double quat_y = msg->pose.orientation.y;
        double quat_z = msg->pose.orientation.z;
        double quat_w = msg->pose.orientation.w;

        double siny_cosp = 2.0 * (quat_w * quat_z + quat_x * quat_y);
        double cosy_cosp = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z);
        double yaw = atan2(siny_cosp, cosy_cosp);

        //Extract position and velocity readings
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
	
        if (firstMsg) {
            initial_x = x;
            initial_y = y;
            initial_yaw = yaw;
	    
	        firstMsg = false;
        }

        x_vicon = x;
        y_vicon = y;
        x_c = x - initial_x;
        y_c = y - initial_y;
        yaw_c = yaw;

        double raw_Vx = (x_vicon - last_x)/0.0334;
        double raw_Vy = (y_vicon - last_y)/0.0334;

        //Correction of the velocity to decrease the noise caused by the derivative calculation of the positions
        double alpha = 0.6;

        V_x = alpha * V_x + (1-alpha) * raw_Vx;
        V_y = alpha * V_y + (1-alpha) * raw_Vy;

    }


     void start() {
        
        if (!initialized) {
 
            //Create the path
            path_vector = create_circle_vector(0.1, 0.1, 0.8, num_points);
            
            // Make the robot stand up
            motion_msgs::msg::MotionCtrl cmd;
            cmd.mode_mark = true;
            cmd.value.up = 1.0;
            cmd.mode.stand_mode = true;
            motion_publisher_->publish(cmd);
            
            initialized = true;
            RCLCPP_INFO(this->get_logger(), "Path is created");
        }
        
        move_to (x_c, y_c, V_x, V_y);

    }

    std::vector<std::pair<double, double>> create_circle_vector(double x_center, double y_center, double radius, int num_points) {
        std::vector<std::pair<double, double>> circle_vector;
        circle_vector.reserve(num_points); // Reserve space for num_points elements
        
        for (int i = 0; i < num_points; ++i) {
	   // Calculate the angle for this point
            double angle = 2 * M_PI * i / num_points;
            
            // Adjust the radius to create petal effects
            // The '4' controls the number of petals
            // Adjusting the amplitude (e.g., radius * 0.2) controls the "softness" of the petals
            double adjusted_radius = radius + radius * 0.2 * std::cos(4 * angle);
            
            // Calculate the position using the adjusted radius
            //double x = x_center + adjusted_radius * std::cos(angle);
            //double y = y_center + adjusted_radius * std::sin(angle);
            double x = x_center + radius * 1 * std::cos(2 * M_PI * i / num_points);
            double y = y_center + radius * 1 * std::sin(2 * M_PI * i / num_points);
            // + 0.8* std::cos(2* 2 * M_PI * i / num_points)
            // + 0.05* std::sin(2* 2 * M_PI * i / num_points)

            circle_vector.emplace_back(x, y); 

        }
        
        return circle_vector;
    }

    pair<double, double> calculateDesiredVelocity(const vector<pair<double, double>>& path_vector, double x_c, double y_c) {
        //Calculate the Shortest distance to the path
        shortest_d = numeric_limits<double>::max();
        double distance = 0;
	
	    vector<double> all_distance = {};
        double h = 0.01;        

        for (size_t i = 0; i < path_vector.size(); ++i) {
            distance = sqrt(pow(path_vector[i].first - x_c, 2) + pow(path_vector[i].second - y_c, 2));
            if (distance < shortest_d) {
                shortest_d = distance;
            }
            all_distance.push_back(distance);
        }

        vector<double> all_rho = {};
        double rho_sum = 0;
        double N_x=0;
        double N_y=0;
        double T_x=0;
        double T_y=0;
        double T2_x=0;
        double T2_y=0;
        double aux_rho;
        curv = 0;

        for(int i=0; i < path_vector.size(); i++)
        {
            aux_rho = exp(-(all_distance[i]-shortest_d)/h);
            N_x += aux_rho*(path_vector[i].first - x_c)/(all_distance[i]+1e-5);
            N_y += aux_rho*(path_vector[i].second - y_c)/(all_distance[i]+1e-5);

            double Tx_a = (path_vector[(i+1)%path_vector.size()].first - path_vector[i].first);
            double Ty_a = (path_vector[(i+1)%path_vector.size()].second - path_vector[i].second);
            double T_norm = sqrt(Tx_a*Tx_a + Ty_a*Ty_a);

            T_x += aux_rho*Tx_a/(T_norm+1e-5);
            T_y += aux_rho*Ty_a/(T_norm+1e-5);

	        //find the next tangent vector for curvature calculation
            double Tx_a2 = (path_vector[(i+2)%path_vector.size()].first - path_vector[(i+1)%path_vector.size()].first);
            double Ty_a2 = (path_vector[(i+2)%path_vector.size()].second - path_vector[(i+1)%path_vector.size()].second);
            double T2_norm = sqrt(Tx_a2*Tx_a2 + Ty_a2*Ty_a2);

            T2_x += aux_rho*Tx_a2/(T2_norm+1e-5);
            T2_y += aux_rho*Ty_a2/(T2_norm+1e-5);

            curv += aux_rho*( Tx_a *(Ty_a2-Ty_a) - Ty_a*(Tx_a2-Tx_a))/pow(T_norm,3);
            
            rho_sum +=  aux_rho;

        }
	    curv = curv/rho_sum;
        N_x = N_x/rho_sum;
        N_y = N_y/rho_sum;
        T_x = T_x/rho_sum;
        T_y = T_y/rho_sum;

        RCLCPP_INFO(this->get_logger(), "Distance to the path is: %.2f \n",shortest_d);

        // Calculate the linear combination weights
        //The parameter 120 was determined based on experiments
        double alpha = (2/M_PI) * atan(120*pow(shortest_d,2));
        double beta = sqrt(1-pow(alpha,2));

        // Calculate the desired velocity components
        double v_max = 0.3;
        double curv_max = 1;
        double mult = 0;

        //To decrease the velocity of the robot at high curvature parts of the path
        if (curv < curv_max) {
            mult = v_max;
        } else if (curv > 9) {
            mult = v_max * 0.2;
        } else {
            mult = (2*curv_max*v_max)/(curv+curv_max);
        }

        //mult depends on the curvature, represents the linear v
        double Dv_x = mult * ((alpha * N_x) + (beta * T_x));
        double Dv_y = mult * ((alpha * N_y) + (beta * T_y));
        
        return {Dv_x, Dv_y};
    }

    void move_to (double xc, double yc, double V_x, double V_y) {

        //Get VF(x,y) and VF(x+u, y) and VF(x, y+u)
        double u = 0.01;
        pair<double, double> Vd = calculateDesiredVelocity(path_vector, xc, yc);
        double Vd_x = Vd.first;
        double Vd_y = Vd.second;

        pair<double, double> Vdx = calculateDesiredVelocity(path_vector, xc+u, yc);
        double Vdx_xp = Vdx.first;
        double Vdx_yp = Vdx.second;

        pair<double, double> Vdy = calculateDesiredVelocity(path_vector, xc, yc+u);
        double Vdy_xp = Vdy.first;
        double Vdy_yp = Vdy.second;

        pair<double, double> Vdx_n = calculateDesiredVelocity(path_vector, xc-u, yc);
        double Vdx_xn = Vdx_n.first;
        double Vdx_yn = Vdx_n.second;

        pair<double, double> Vdy_n = calculateDesiredVelocity(path_vector, xc, yc-u);
        double Vdy_xn = Vdy_n.first;
        double Vdy_yn = Vdy_n.second;

        
        //Calculate Jacobian Matrix
        double J[2][2];

        J[0][0] = (Vdx_xp - Vdx_xn)/(2*u);
        J[1][0] = (Vdx_yp - Vdx_yn)/(2*u);
        J[0][1] = (Vdy_xp - Vdy_xn)/(2*u);
        J[1][1] = (Vdy_yp - Vdy_yn)/(2*u);

        //Save the current velocity vector into a Matrix for matrix multiplication
        double v[2][1];
        v[0][0] = V_x;
        v[1][0] = V_y;
  
        //Perform Matrix multiplication to Obtain Ad_x, Ad_y
        double Ad_x = (J[0][0] * v[0][0]) + (J[0][1] * v[1][0]);
        double Ad_y = (J[1][0] * v[0][0]) + (J[1][1] * v[1][0]);

        //Perfom error correction to obtain Ax, Ay
        double c_factor = 1; //0.2
        double Ax = Ad_x - (c_factor * (V_x - Vd_x));
        double Ay = Ad_y - (c_factor * (V_y - Vd_y));
	
        double normA  = sqrt(Ax*Ax+Ay*Ay);
        if (normA >= 0.3)
        {
            Ax = (0.3*Ax)/normA;
            Ay = (0.3*Ay)/normA;
        }        

        //Find V_dot and omega
        double V_dot = (cos(yaw_c) * Ax) + (sin(yaw_c) * Ay);
        //Calculare linear velocity from v_dot
        linear_v = last_v + (V_dot * 0.1);

        //at the first few seconds, the angular_v is too large because the last_v is too small 
        double angular_v = 0;
        if (abs(last_v) >= 0.05) {
            angular_v = (((-1) * sin(yaw_c) * Ax) + (cos(yaw_c) * Ay)) / (last_v); 
        } else {
            angular_v = (((-1) * sin(yaw_c) * Ax) + (cos(yaw_c) * Ay))/(0.05*last_v/abs(last_v));
        }
        
        //Override for a test
        double e = 0.1;
        //linear_v = ((cos(yaw_c) * Vd_x) + (sin(yaw_c) * Vd_y));
        //angular_v = ((-1* sin(yaw_c) * Vd_x) + (cos(yaw_c) * Vd_y))/e;

        double fact = 1;
        if(abs(angular_v) > 1) {
            fact = 1/abs(angular_v);
        } 
        angular_v *= fact;
        linear_v *= fact; 

        std::ofstream outfile;
        outfile.open(file_path_, std::ios_base::app);
        
        // Write data in columns separated by commas
        rclcpp::Time now = rclcpp::Clock().now();
        int64_t milliseconds = now.nanoseconds() / 1000000;

        if (first_loop) {
            //record the path vector
            for (size_t i = 0; i < path_vector.size(); ++i) {
                outfile << path_vector[i].first << ",";
                outfile << path_vector[i].second << endl;
            }

            //add the titles to the data  
            outfile << "Timestamp_millisec,Position_X,Position_Y,Orientation_Z, current_Vel_x, current_Vel_y, desired_Velocity_x, desired_Velocity_y, Target linear_velocity, Target Angular_v, Target Accel, x_vicon, y_vicon, yaw_vicon, Vx_vicon, Vy_vicon " << std::endl;

            first_loop = false;
        }

        outfile << milliseconds << ",";
        outfile << x_c << ",";
        outfile << y_c << ",";
        outfile << yaw_c << ",";
	    outfile << V_x << ",";
	    outfile << V_y << ",";
        outfile << Vd_x << ",";
        outfile << Vd_y << ",";
        outfile << linear_v << ",";
        outfile << angular_v << ",";
        outfile << V_dot << ",";
        outfile << x_vicon << ",";
        outfile << y_vicon << ",";
        outfile << yaw_vicon << ",";
	    outfile << V_x_vicon << ",";
        outfile << V_y_vicon << std::endl;

        outfile.close();
        
        //Send the commands to the robot
        motion_msgs::msg::MotionCtrl cmd;
        cmd.mode_mark = false;
        cmd.value.forward = linear_v;
        cmd.value.left = angular_v;
        cmd.value.up = 1;
        motion_publisher_->publish(cmd);

        last_v = linear_v;
        last_omega = angular_v;

	

    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr diablo_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pose_subscriber_;
    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    bool  firstMsg;
    double initial_x_vicon, initial_x;
    double initial_y_vicon, initial_y;
    double initial_yaw;
    double Kv;
    double Kw;
    double last_omega;
    double x_c, y_c, yaw_c, yaw_vicon, V_x, V_y, x_vicon, y_vicon;
    double V_x_vicon, V_y_vicon;
    double last_v, last_x, last_y, last_x_vicon, last_y_vicon;
    double Dv_y, Dv_x;
    double target_x, target_y;
    double shortest_d;
    vector<pair<double, double>> path_vector;
    int num_points;
    bool initialized;
    double linear_v;
    std::string file_path_;
    bool first_loop;
    double curv;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloSquarePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

