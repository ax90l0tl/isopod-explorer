#include "raspi_dma_pwm.h"
#include <iostream>
using namespace std;

PWM_obj::PWM_obj(int pigpio_id, uint8_t pin, int min, int max, int frequency)
{
    // cout << "initializing" << endl;
    // pi = pigpio_start(NULL, NULL);
    pin_ = pin;
    frequency_ = frequency;
    min_ = min;
    max_ = max;
    mid_ = ((max - min) / 2) + min;
    pi_ = pigpio_id;
    // cout << "initialize complete" << endl;
    set_mode(pi_, pin_, PI_OUTPUT);
    set_PWM_frequency(pi_, pin_, frequency);
    if (get_PWM_range(pi_, pin_) != 255)
    {
        set_PWM_range(pi_, pin_, 255);
    }
    run(0);
}

PWM_obj::~PWM_obj()
{
    cout << "destructing" << endl;
    run(0);
}

void PWM_obj::run(float speed)
{
    if (speed > 1)
    {
        speed = 1;
    }
    else if (speed < -1)
    {
        speed = -1;
    }
    int pulse = (speed * (max_ - min_) / 2) + mid_;
    uint duty_cycle = pulse * frequency_ * 255 * 1e-6;
    cout << "duty cycle: " << duty_cycle << " "
         << "pulse: " << pulse << endl;
    set_PWM_dutycycle(pi_, pin_, duty_cycle);
}