#ifndef COMMONFUNC_HPP
#define COMMONFUNC_HPP

#include <iostream>
#include <cmath>

namespace CommonFunc{
    // Linear interpolation function
    double lerp(double x0, double x1, double t);
    double map(double value, double in_min, double in_max, double out_min, double out_max);
}

#endif