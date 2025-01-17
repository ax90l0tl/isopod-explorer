#ifndef SIM_COMMS_NODE_HPP
#define SIM_COMMS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rov_msgs/msg/thruster_command.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <cmath>
#include <chrono>
#include <functional>
#include <vector>
#include <memory>
#include <iostream>
using namespace std;

class Sim_comms_node : public rclcpp::Node
{
public:
    Sim_comms_node();

private:
    vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> thruster_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_vel_pub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
    rclcpp::Subscription<rov_msgs::msg::ThrusterCommand>::SharedPtr thruster_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    void odom_Callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void thruster_Callback(const rov_msgs::msg::ThrusterCommand::SharedPtr msg);
    void depth_Callback();
    void gps_Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr depth_timer;
    nav_msgs::msg::Odometry odom;

    double depth_cov;
    vector<double> gps_cov;
};

#endif