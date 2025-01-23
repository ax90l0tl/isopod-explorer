#ifndef CONTROLLER
#define CONTROLLER

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "rov_msgs/msg/command.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <algorithm>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rov_control
{
  class PID_6DOF : public controller_interface::ChainableControllerInterface
  {
  public:
    PID_6DOF();
    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
  private:
  };
}

#endif