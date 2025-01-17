#include "sim_comms.hpp"

using std::placeholders::_1;

Sim_comms_node::Sim_comms_node() : rclcpp::Node("sim_comms")
{
    // Declare Parameters
    this->declare_parameter("depth_rate", 30);
    this->declare_parameter("depth_topic", "depth_sensor");
    this->declare_parameter("gps_topic", "gps/ros");
    this->declare_parameter("gps_vel_topic", "gps/vel/ros");
    this->declare_parameter("gps_topic_gz", "gps");
    this->declare_parameter("odom_topic", "odom");
    this->declare_parameter("thruster_topic", "thruster_command");
    this->declare_parameter("depth_cov", 0.00004);
    this->declare_parameter("gps_cov", std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5});
    this->declare_parameter("n_thrusters", 6);

    depth_cov = this->get_parameter("depth_cov").as_double();
    gps_cov = this->get_parameter("gps_cov").as_double_array();
    int period = 1000 / this->get_parameter("depth_rate").as_int();
    depth_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(this->get_parameter("depth_topic").as_string(), 1);
    depth_timer = this->create_wall_timer(chrono::milliseconds(period), bind(&Sim_comms_node::depth_Callback, this));

    gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(this->get_parameter("gps_topic").as_string(), 1);
    gps_vel_pub = this->create_publisher<nav_msgs::msg::Odometry>(this->get_parameter("gps_vel_topic").as_string(), 1);
    thruster_pub.resize(this->get_parameter("n_thrusters").as_int());
    for (uint8_t i = 0; i < this->get_parameter("n_thrusters").as_int(); i++)
    {
        thruster_pub[i] = this->create_publisher<std_msgs::msg::Float64>(string("thruster" + to_string(i)), 10);
    }

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic").as_string(), 10, std::bind(&Sim_comms_node::odom_Callback, this, _1));
    thruster_sub = this->create_subscription<rov_msgs::msg::ThrusterCommand>(this->get_parameter("thruster_topic").as_string(), 10, std::bind(&Sim_comms_node::thruster_Callback, this, _1));
    gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(this->get_parameter("gps_topic_gz").as_string(), 10, std::bind(&Sim_comms_node::gps_Callback, this, _1));
}

void Sim_comms_node::odom_Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom = *msg;
}

void Sim_comms_node::thruster_Callback(const rov_msgs::msg::ThrusterCommand::SharedPtr msg)
{
    auto msg_out = std_msgs::msg::Float64();
    for (uint8_t i = 0; i < msg->thrusters.size(); i++)
    {
        if (thruster_pub[i] == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Publisher at index %zu is invalid (nullptr)", size_t(i));
        }
        else
        {
            msg_out.data = msg->thrusters[i];
            thruster_pub[i]->publish(msg_out);
        }
    }
}

void Sim_comms_node::depth_Callback()
{
    // Has to be a pose message for the robot localization ekf
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "Bar30";
    // Covariance values are taken from https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1/
    // but not sure how correctly I filled out the matrix
    msg.pose.pose.position.z = odom.pose.pose.position.z;
    msg.pose.covariance = {0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, depth_cov, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0};
    depth_pub->publish(msg);
}

void Sim_comms_node::gps_Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (odom.pose.pose.position.z >= -0.5)
    {
        auto msg_out = sensor_msgs::msg::NavSatFix();
        msg_out = *msg;
        std::copy(gps_cov.begin(), gps_cov.end(), msg_out.position_covariance.begin());
        msg_out.position_covariance_type = 2;
        gps_pub->publish(msg_out);
        gps_vel_pub->publish(odom);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim_comms_node>());
    rclcpp::shutdown();
    return 0;
}