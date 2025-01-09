#include "thruster_manager.hpp"
using std::placeholders::_1;
using namespace std;
Thruster_manager::Thruster_manager() : rclcpp::Node("thruster_manager")
{
    // Parameters
    this->declare_parameter("wrench_sub_topic", "wrench");
    this->declare_parameter("cmd_sub_topic", "cmd");
    this->declare_parameter("thrust_cmd_pub_topic", "thrust_cmd");
    this->declare_parameter("thrust_max_fwd", 5.25);
    this->declare_parameter("thrust_max_bwd", 4.1);
    this->declare_parameter("thrust_deadband", 0.000001);
    this->declare_parameter("motor_driver_deadband", 0.0625);
    this->declare_parameter("rate_limit", 0.1);
    this->declare_parameter("max_force", 60.0);
    this->declare_parameter("max_torque", 80.0);
    this->declare_parameter("num_motors", 8);
    // Parameter Lists
    const std::map<std::string, double> &motor = {{"surge", 0.0}, {"sway", 0.0}, {"heave", 0.0}, {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0}};
    for (int i = 0; i < this->get_parameter("num_motors").as_int(); i++)
    {
        this->declare_parameters(string("motor" + to_string(i)), motor);
    }

    cmd_sub = this->create_subscription<rov_msgs::msg::Command>(
        this->get_parameter("cmd_sub_topic").as_string(), 1, std::bind(&Thruster_manager::cmd_Callback, this, _1));
    wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        this->get_parameter("wrench_sub_topic").as_string(), 1, std::bind(&Thruster_manager::wrench_Callback, this, _1));
    thrust_cmd_pub = this->create_publisher<rov_msgs::msg::ThrusterCommand>(
        this->get_parameter("thrust_cmd_pub_topic").as_string(), 1);
    setVariables();
}

Thruster_manager::~Thruster_manager()
{
    RCLCPP_INFO(this->get_logger(), "Destructing, stopping all thrusters");
    power_off();
}

void Thruster_manager::power_off()
{
    output.thrusters.clear();
    for (uint8_t i = 0; i < num_motors; i++)
    {
        output.thrusters.push_back(0);
    }
    output.auxilary.clear();
    output.buttons.clear();
    for (uint8_t i = 0; i < 8; i++)
    {
        output.auxilary.push_back(0);
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        output.buttons.push_back(0);
    }
    thrust_cmd_pub->publish(output);
}

void Thruster_manager::setVariables()
{
    THRUST_MAX_FWD = this->get_parameter("thrust_max_fwd").as_double();
    THRUST_MAX_BWD = this->get_parameter("thrust_max_bwd").as_double();
    MOTOR_FORWARD_BACKWARD_RATIO = THRUST_MAX_BWD / THRUST_MAX_FWD;
    THRUST_DEADBAND_EPS = this->get_parameter("thrust_deadband").as_double();
    MOTOR_DRIVER_DEADBAND = this->get_parameter("motor_driver_deadband").as_double();
    FORCE_MAX = this->get_parameter("max_force").as_double();
    TORQUE_MAX = this->get_parameter("max_torque").as_double();
    THRUST_MAX_FWD_N = THRUST_MAX_FWD * KGF2N;
    THRUST_MAX_BWD_N = THRUST_MAX_BWD * KGF2N;
    max_step_per_loop = this->get_parameter("rate_limit").as_double();
    num_motors = this->get_parameter("num_motors").as_int();

    last_motor_command.resize(num_motors, 0);
    motor_command.resize(num_motors, 0);
    for (int i = 0; i < num_motors; ++i)
    {
        motors[i]["surge"] = this->get_parameter(string("motor" + to_string(i) + ".surge")).as_double();
        motors[i]["sway"] = this->get_parameter(string("motor" + to_string(i) + ".sway")).as_double();
        motors[i]["heave"] = this->get_parameter(string("motor" + to_string(i) + ".heave")).as_double();
        motors[i]["roll"] = this->get_parameter(string("motor" + to_string(i) + ".roll")).as_double();
        motors[i]["pitch"] = this->get_parameter(string("motor" + to_string(i) + ".pitch")).as_double();
        motors[i]["yaw"] = this->get_parameter(string("motor" + to_string(i) + ".yaw")).as_double();
    }
}

void Thruster_manager::cmd_Callback(const rov_msgs::msg::Command::SharedPtr msg)
{
    this->output.auxilary.clear();
    this->output.buttons.clear();
    for (uint8_t i = 0; i < msg->auxilary.size(); i++)
    {
        this->output.auxilary.push_back(msg->auxilary[i]);
    }
    for (uint8_t i = 0; i < msg->buttons.size(); i++)
    {
        this->output.buttons.push_back(msg->buttons[i]);
    }
}

void Thruster_manager::wrench_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    // Assemble desired commands, ensure proper ranges and convert to force/torque requests
    // DOF order: surge, sway, heave, roll, pitch, yaw)
    std::map<std::string, double> des_forces;
    des_forces["surge"] = std::clamp(msg->wrench.force.x, -FORCE_MAX, FORCE_MAX);
    des_forces["sway"] = std::clamp(msg->wrench.force.y, -FORCE_MAX, FORCE_MAX);
    des_forces["heave"] = std::clamp(msg->wrench.force.z, -FORCE_MAX, FORCE_MAX);
    des_forces["roll"] = std::clamp(msg->wrench.torque.x, -TORQUE_MAX, TORQUE_MAX);
    des_forces["pitch"] = std::clamp(msg->wrench.torque.y, -TORQUE_MAX, TORQUE_MAX);
    des_forces["yaw"] = std::clamp(msg->wrench.torque.z, -TORQUE_MAX, TORQUE_MAX);
    // Loop through each motor
    for (size_t i = 0; i < motors.size(); ++i)
    {
        motor_command[i] = 0;
        std::map<std::string, double> motor = motors[i];
        // Loop through each direction
        for (auto const &DOF : constants::DOFs)
        {
            motor_command[i] += des_forces[DOF] * motor[DOF];
        }
    }
    // Convert motor thrusts to commands
    std::vector<float> motor_comms(motors.size(), 0);
    output.thrusters.clear();
    // Publish message
    for (size_t i = 0; i < motors.size(); ++i)
    {
        // double m_comms = thrust_to_motor_comm(motor_command[i]);
        // motor_comms[i] = rateLimitMotorCommand(m_comms, last_motor_command[i]);
        motor_comms[i] = motor_command[i];
        output.thrusters.push_back(motor_comms[i]);
        // Store the last command so we can ramp it
        last_motor_command[i] = motor_comms[i];
    }

    thrust_cmd_pub->publish(output);
}

// Takes in thrust command in Newtons and returns motor commands in [-1,1] scaled accordingly with deadbands
double Thruster_manager::thrust_to_motor_comm(const double thrust_n)
{
    // Returns motor comms
    if (abs(thrust_n) < THRUST_DEADBAND_EPS)
    {
        return 0;
    }
    else if (thrust_n > 0)
    {
        return CommonFunc::map(thrust_n, 0, THRUST_MAX_FWD_N, MOTOR_DRIVER_DEADBAND, 1.0);
    }
    else
    {
        return CommonFunc::map(thrust_n, 0, -THRUST_MAX_BWD_N, -MOTOR_DRIVER_DEADBAND, -1.0);
    }
} // End of thrust_to_motor function

double Thruster_manager::rateLimitMotorCommand(double new_command, double last_command) const
{
    if (abs(last_command) < MOTOR_DRIVER_DEADBAND && abs(new_command) >= MOTOR_DRIVER_DEADBAND)
    {
        return std::copysign(MOTOR_DRIVER_DEADBAND, new_command);
    }
    else
    {
        return std::clamp(new_command, last_command - max_step_per_loop, last_command + max_step_per_loop); // Rate limit
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Thruster_manager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}