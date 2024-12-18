#include "sim_comms.hpp"

using std::placeholders::_1;

Sim_comms_node::Sim_comms_node() : rclcpp::Node("sim_comms")
{
    // Declare Parameters
    this->declare_parameter("depth_rate", 30);
    this->declare_parameter("gps_rate", 5);
    this->declare_parameter("depth_topic", "depth_sensor");
    this->declare_parameter("gps_topic", "gps");
    this->declare_parameter("odom_topic", "odom");
    this->declare_parameter("depth_mean", 0.0);
    this->declare_parameter("depth_variance", 0.0);
    this->declare_parameter("gps_mean", 0.0);
    this->declare_parameter("gps_variance", 0.0);
    this->declare_parameter("n_thrusters", 6);

    int period = 1000 / this->get_parameter("depth_rate").as_int();
    depth_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(this->get_parameter("depth_topic").as_string(), 10);
    depth_timer = this->create_wall_timer(chrono::milliseconds(period), bind(&Sim_comms_node::depth_Callback, this));

    period = 1000 / this->get_parameter("gps_rate").as_int();
    gps_pub = this->create_publisher<nav_msgs::msg::Odometry>(this->get_parameter("gps_topic").as_string(), 10);
    gps_timer = this->create_wall_timer(chrono::milliseconds(period), bind(&Sim_comms_node::gps_Callback, this));

    for (uint8_t i = 0; i < this->get_parameter("n_thrusters").as_int(); i++)
    {
        thruster_pub.push_back(this->create_publisher<std_msgs::msg::Float64>(string("thruster" + to_string(i)), 10));
    }

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic").as_string(), 10, std::bind(&Sim_comms_node::odom_Callback, this, _1));
}

void Sim_comms_node::odom_Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom = *msg;
}

void Sim_comms_node::depth_Callback()
{
    // Has to be a pose message for the robot localization ekf
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "depth_sensor";
    // Covariance values are taken from https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1/
    // but not sure how correctly I filled out the matrix
    msg.pose.pose.position.z = odom.pose.pose.position.z;
    msg.pose.covariance = {0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, pow(0.002, 2), 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0};
    depth_pub->publish(msg);
}

void Sim_comms_node::gps_Callback()
{
    if (odom.pose.pose.position.z >= 0.1)
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "gps";
        msg.pose = odom.pose;
        msg.twist = odom.twist;
        gps_pub->publish(msg);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim_comms_node>());
    rclcpp::shutdown();
    return 0;
}