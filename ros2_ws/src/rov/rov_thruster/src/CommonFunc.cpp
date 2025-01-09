#include "CommonFunc.hpp"

double CommonFunc::lerp(double x0, double x1, double t){
        return x0 + t * (x1 - x0);
}

double CommonFunc::map(double value, double in_min, double in_max, double out_min, double out_max)
{
    double in_span = in_max - in_min;
    double out_span = out_max - out_min;

    double scaled = double(value - in_min) / double(in_span);
    return out_min + (scaled * out_span);
}