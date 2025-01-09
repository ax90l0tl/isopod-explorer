#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "MTi.h"
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
using namespace std;

class Imu_node : public rclcpp::Node
{
public:
    Imu_node();
    ~Imu_node();

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    // comment out if you don't want to see euler data
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr eul_pub_;
    // North(x)-East(y)-Down(z) Coordinate
    void timer_Callback();
    rclcpp::TimerBase::SharedPtr timer_;
    unique_ptr<MTi> imu_;
    int pi_;
};

#endif