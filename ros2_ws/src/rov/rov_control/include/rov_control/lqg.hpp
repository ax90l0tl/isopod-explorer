#ifndef LQG_HPP
#define LQG_HPP

#include "controller_interface/controller_interface.hpp"

namespace lqg
{
    class LQG : public controller_interface::ControllerInterface
    {
    public:
        LQG();
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;
    };

} // namespace lqg

#endif // LQG_HPP