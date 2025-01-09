#ifndef RASPI_DMA_PWM_HPP
#define RASPI_DMA_PWM_HPP
#include <pigpiod_if2.h>

class PWM_obj
{
public:
    PWM_obj(int pigpio_id, uint8_t pin, int min = 1100, int max = 1900, int frequency = 200);
    ~PWM_obj();
    void run(float speed);

private:
    uint8_t pi_;
    uint8_t pin_;
    int min_;
    int max_;
    int mid_;
    int frequency_;
};

#endif