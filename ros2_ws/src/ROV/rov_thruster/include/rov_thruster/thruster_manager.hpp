#ifndef THRUSTER_MANAGER_H
#define THRUSTER_MANAGER_H

#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rov_msgs/msg/thruster_command.hpp"
#include <iostream>
#include <memory>
using namespace std;

class Thruster_manager : public rclcpp::Node
{
public:
    Thruster_manager();

private:
    void sub_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    rclcpp::Publisher<rov_msgs::msg::ThrusterCommand>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
};


#endif