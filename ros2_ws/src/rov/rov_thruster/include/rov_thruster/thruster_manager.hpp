#ifndef THRUSTER_MANAGER_HPP
#define THRUSTER_MANAGER_HPP

#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>
#include <array>
#include <vector>
#include <map>
#include <memory>
#include "CommonFunc.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rov_msgs/msg/thruster_command.hpp"
#include "rov_msgs/msg/command.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
// #include <eigen3/Dense>
// #include <eigen3/Sparse>

using namespace std;

#define SURGE 0
#define SWAY 1
#define HEAVE 2
#define ROLL 3
#define PITCH 4
#define YAW 5

//

namespace constants
{
    const std::array<std::string, 6> DOFs = {"surge", "sway", "heave", "roll", "pitch", "yaw"};
}

class Thruster_manager : public rclcpp::Node
{
public:
    Thruster_manager();
    ~Thruster_manager();

private:
    // Functions
    
    double thrust_to_motor_comm(const double thrust_n);
    void setVariables();
    void allocate_generic_motors(std::map<std::string, double> &des_forces, std::vector<double> &des_motor_thrusts);
    double rateLimitMotorCommand(double new_command, double last_command) const;
    void cmd_Callback(const rov_msgs::msg::Command::SharedPtr msg);
    void wrench_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void power_off();
    // Ros stuff
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
    rclcpp::Subscription<rov_msgs::msg::Command>::SharedPtr cmd_sub;
    rclcpp::Publisher<rov_msgs::msg::ThrusterCommand>::SharedPtr thrust_cmd_pub;
    rov_msgs::msg::ThrusterCommand output;

    // Variables
    map<int, std::map<std::string, double>> motors;
    int8_t num_motors;
    vector<double> motor_command;
    vector<double> last_motor_command;

    double max_step_per_loop;
    double MOTOR_FORWARD_BACKWARD_RATIO; // BlueROV T200 produce 5.25kgf forward, 4.1 kgf backwards, at 16V
    double THRUST_MAX_FWD;               // kg-f @ 16V
    double THRUST_MAX_BWD;               // kg-f @ 16V
    double KGF2N = 9.80665;              // kg-f to Newton ant
    double THRUST_DEADBAND_EPS;
    double MOTOR_DRIVER_DEADBAND; // Scale from 0 to 1
    double FORCE_MAX;      // N
    double TORQUE_MAX;      // N-m
    double THRUST_MAX_FWD_N;
    double THRUST_MAX_BWD_N;

    bool test_mode = false;
};

#endif