#include "joy_command.hpp"

Joy_cmd::Joy_cmd() : rclcpp::Node("joy_cmd")
{
    this->declare_parameter("pub_topic", "command");
    this->declare_parameter("sub_topic", "joy");
    pub_ = this->create_publisher<rov_msgs::msg::Command>(this->get_parameter("pub_topic").as_string(), 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(this->get_parameter("sub_topic").as_string(), 10,
                                                            bind(&Joy_cmd::joy_cb, this, placeholders::_1));
}

void Joy_cmd::joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    auto output = rov_msgs::msg::Command();
    output.command.force.x = msg->axes[2];
    output.command.force.y = msg->axes[3];
    output.command.force.z = msg->axes[0];
    output.command.torque.x = msg->axes[5];
    output.command.torque.y = msg->axes[6];
    output.command.torque.z = msg->axes[1];
    for(uint8_t i = 0; i < msg->buttons.size(); i++){
        output.buttons.push_back(msg->buttons[i]);
    }
    pub_->publish(output);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joy_cmd>());
    rclcpp::shutdown();
    return 0;
}