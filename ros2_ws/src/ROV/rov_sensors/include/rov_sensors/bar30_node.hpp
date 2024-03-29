#ifndef BAR30_NODE_HPP
#define BAR30_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "MS5837.h"
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
using namespace std;

class Bar30_node : public rclcpp::Node
{
public:
    Bar30_node();
    ~Bar30_node();

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    // North(x)-East(y)-Down(z) Coordinate
    void timer_Callback();
    rclcpp::TimerBase::SharedPtr timer_;
    unique_ptr<MS5837> sensor_;
    int pi_;
};

#endif