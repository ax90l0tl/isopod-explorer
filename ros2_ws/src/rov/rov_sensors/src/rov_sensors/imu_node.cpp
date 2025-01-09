#include "imu_node.hpp"

using std::placeholders::_1;

Imu_node::Imu_node() : rclcpp::Node("imu_")
{
    this->declare_parameter("pi_address", "192.168.8.157");
    this->declare_parameter("pi_port", "8888");
    this->declare_parameter("pub_topic", "imu_data");
    this->declare_parameter("rate", 30);

    pi_ = pigpio_start(this->get_parameter("pi_address").as_string().c_str(),
                       this->get_parameter("pi_port").as_string().c_str());
    if (pi_ < 0)
    {
        RCLCPP_INFO(this->get_logger(), "PI not detected!");
    }
    cout << "pigpio_id " << pi_ << endl;
    // create a dummy pointer for the class obj imu_
    unique_ptr<MTi> dummy(new MTi(pi_));
    imu_ = move(dummy);
    if (imu_->detect(1000))
    {
        // Configure imu
        const string modes[] = {"QUATERNION", "RATEOFTURNHR", "ACCELERATION", "EULER"};
        int rate = this->get_parameter("rate").as_int();
        // Can also do separate rates for each datatype
        const uint16_t hz[] = {rate, rate, rate, rate};
        imu_->configureOutputs(modes, hz, 4);

        int n = sizeof(hz) / sizeof(hz[0]);
        int period = 0.5 * (1000 / (*max_element(hz, hz + n)));

        pub_ = this->create_publisher<sensor_msgs::msg::Imu>(this->get_parameter("pub_topic").as_string(), 10);
        // comment out if you don't want to see euler data
        eul_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu_euler", 10);
        timer_ = this->create_wall_timer(chrono::milliseconds(period), bind(&Imu_node::timer_Callback, this));
        RCLCPP_INFO(this->get_logger(), "Publishing to topic: imu_data at '%i' hz", 1000 / period);
        imu_->goToMeasurement();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "IMU not detected!");
    }
}

Imu_node::~Imu_node()
{
    RCLCPP_INFO(this->get_logger(), "Destructing");
}

void Imu_node::timer_Callback()
{
    auto msg = sensor_msgs::msg::Imu();
    // imu_->printData();
    imu_->readData();
    msg.header.stamp = now();
    msg.header.frame_id = "imu_data";
    // Covariance values from here https://mtidocs.movella.com/sensor-specifications$sensor-specifications
    // Not sure how correctly I filled out the matrix though
    msg.orientation.w = imu_->getQuat()[0];
    msg.orientation.x = imu_->getQuat()[1];
    msg.orientation.y = imu_->getQuat()[2];
    msg.orientation.z = imu_->getQuat()[3];
    msg.orientation_covariance = {0.0139626, 0, 0,
                                  0, 0.0139626, 0,
                                  0, 0, 0.0349066};

    msg.angular_velocity.x = imu_->getRateOfTurn()[0];
    msg.angular_velocity.y = imu_->getRateOfTurn()[1];
    msg.angular_velocity.z = imu_->getRateOfTurn()[2];
    msg.angular_velocity_covariance = {0.007330383, 0, 0,
                                       0, 0.007330383, 0,
                                       0, 0, 0.007330383};

    msg.linear_acceleration.x = imu_->getAcceleration()[0];
    msg.linear_acceleration.y = imu_->getAcceleration()[1];
    msg.linear_acceleration.z = imu_->getAcceleration()[2];
    msg.linear_acceleration_covariance = {0.07056, 0, 0,
                                          0, 0.07056, 0,
                                          0, 0, 0.07056};

    pub_->publish(msg);
    
    // comment out if you don't want to see euler data
    auto msg_euler = geometry_msgs::msg::Vector3();
    msg_euler.x = imu_->getEulerAngles()[0];
    msg_euler.y = imu_->getEulerAngles()[1];
    msg_euler.z = imu_->getEulerAngles()[2];

    eul_pub_->publish(msg_euler);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Imu_node>());
    rclcpp::shutdown();
    return 0;
}