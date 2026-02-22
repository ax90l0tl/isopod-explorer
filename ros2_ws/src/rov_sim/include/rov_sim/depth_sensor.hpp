#ifndef DEPTH_SENSOR_HPP
#define DEPTH_SENSOR_HPP

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>

namespace rov_sim
{
    class Depth_Sensor : public gz::sensors::Sensor
    {
        /// \brief Load the sensor with SDF parameters.
        /// \param[in] _sdf SDF Sensor parameters.
        /// \return True if loading was successful
    public:
        virtual bool Load(const sdf::Sensor &_sdf) override;

        /// \brief Update the sensor and generate data
        /// \param[in] _now The current time
        /// \return True if the update was successfull
        virtual bool Update(const std::chrono::steady_clock::duration &_now) override;

        /// \brief Get the latest world postiion of the robot.
        /// \return The latest position given to the sensor.
        const gz::math::Vector3d &Position() const;

        /// \brief Previous position of the robot.
    private:
        /// \brief Latest total distance.
        double depth{0.0};

        /// \brief Noise that will be applied to the sensor data
        gz::sensors::NoisePtr noise{nullptr};

        /// \brief Node for communication
        gz::transport::Node node;

        /// \brief Publishes sensor data
        gz::transport::Node::Publisher pub;
    }
}

#endif // DEPTH_SENSOR_HPP