#ifndef THRUST_ALLOCATOR_HPP
#define THRUST_ALLOCATOR_HPP

#include <iostream>
#include <Eigen/Dense>

namespace thrust_allocator
{
    class ThrustAllocator
    {
    public:
        ThrustAllocator();
        bool readURDF();
        void setHeadlessMode(bool headless);
        Eige n::MatrixXd calculateThrustMatrix();
        Eigen::MatrixXd calculateThrustMatrix(const Eigen::Quaternion<double> &orientation);
        Eigen::MatrixXd allocateThrust(const Eigen::Vector<double, 6> &wrench_cmd);
    private:
        bool headless_;
        Eigen::Quaternion<double> orientation;
        Eigen::MatrixXd thrust_matrix;
        Eigen::MatrixXd thrust_matrix_pseudo_inverse;


    };

} // namespace thrust_allocator

#endif // THRUST_ALLOCATOR_HPP