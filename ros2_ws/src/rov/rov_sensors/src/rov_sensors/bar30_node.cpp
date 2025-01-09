#include "bar30_node.hpp"

using std::placeholders::_1;

Bar30_node::Bar30_node() : rclcpp::Node("bar30")
{
    // Declare Parameters
    this->declare_parameter("pi_address", "192.168.8.157");
    this->declare_parameter("pi_port", "8888");
    this->declare_parameter("rate", 30);
    this->declare_parameter("fluid_density", 1029.0);
    this->declare_parameter("pub_topic", "depth_sensor");

    // Start Pigpio
    pi_ = pigpio_start(this->get_parameter("pi_address").as_string().c_str(),
                      this->get_parameter("pi_port").as_string().c_str());
    if (pi_ < 0)
    {
        RCLCPP_INFO(this->get_logger(), "PI not detected!");
    }
    cout << "pigpio_id " << pi_ << endl;

    // create a dummy pointer for the class obj Bar30
    unique_ptr<MS5837> dummy(new MS5837(pi_));
    sensor_ = move(dummy);
    // Initialize sensor
    if (sensor_->init())
    {
        sensor_->setFluidDensity(this->get_parameter("fluid_density").as_double());
        sensor_->setOSR(5);
        int period = 1000 / this->get_parameter("rate").as_int();
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("depth_sensor", 10);
        timer_ = this->create_wall_timer(chrono::milliseconds(period), bind(&Bar30_node::timer_Callback, this));
        RCLCPP_INFO(this->get_logger(), "Publishing to topic: Bar30_data at '%i' hz", 1000 / period);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Bar30 not detected!");
    }
}

Bar30_node::~Bar30_node()
{
    RCLCPP_INFO(this->get_logger(), "Destructing");
    pigpio_stop(pi_);
}

void Bar30_node::timer_Callback()
{
    // Has to be a pose message for the robot localization ekf
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    // Bar30->printData();
    sensor_->read();
    msg.header.stamp = this->now();
    msg.header.frame_id = "depth_sensor";
    // Covariance values are taken from https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1/
    // but not sure how correctly I filled out the matrix
    msg.pose.pose.position.z = sensor_->depth();
    msg.pose.covariance = {0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0.002, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0};
    pub_->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bar30_node>());
    rclcpp::shutdown();
    return 0;
}