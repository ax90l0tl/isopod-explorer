#ifndef SIM_COMMS_NODE_HPP
#define SIM_COMMS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    void odom_Callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void depth_Callback();
    void gps_Callback();
    rclcpp::TimerBase::SharedPtr depth_timer;
    rclcpp::TimerBase::SharedPtr gps_timer;
    nav_msgs::msg::Odometry odom;
};

#endif