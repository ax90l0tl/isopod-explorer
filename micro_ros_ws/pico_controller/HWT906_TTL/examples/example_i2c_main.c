/**
 * Example showing calibration, settings, and quaternion on Raspberry Pi Pico (I2C)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "jy901.h"

#define I2C_PORT i2c0
#define I2C_SDA  4
#define I2C_SCL  5
#define I2C_ADDR 0x50

static jy901_t imu;

int main()
{
    stdio_init_all();

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    jy901_init_i2c(&imu, I2C_PORT, I2C_ADDR);

    printf("JY901 / HWT906 I2C example\n");

    // --- Example: Configure output rate and enable quaternion ---
    jy901_set_output_rate(&imu, JY901_RATE_100HZ);
    jy901_enable_quaternion(&imu);
    jy901_save(&imu);
    printf("Output rate set to 100 Hz, quaternion enabled, settings saved.\n");

    sleep_ms(500);

    while (true) {
        jy901_get_acc(&imu);
        jy901_get_gyro(&imu);
        jy901_get_angle(&imu);
        jy901_get_quaternion(&imu);

        printf("Acc: %.3f %.3f %.3f g\n",
               jy901_acc_to_g(imu.acc.a[0]),
               jy901_acc_to_g(imu.acc.a[1]),
               jy901_acc_to_g(imu.acc.a[2]));

        printf("Angle: %.2f %.2f %.2f deg\n",
               jy901_angle_to_deg(imu.angle.angle[0]),
               jy901_angle_to_deg(imu.angle.angle[1]),
               jy901_angle_to_deg(imu.angle.angle[2]));

        float q0, q1, q2, q3;
        jy901_quat_to_float(&imu.quat, &q0, &q1, &q2, &q3);
        printf("Quat: %.4f %.4f %.4f %.4f\n", q0, q1, q2, q3);

        printf("----\n");
        sleep_ms(200);
    }

    return 0;
}

/*
 * Calibration example (uncomment and call once when needed):
 *
 * void do_acc_calibration(void) {
 *     printf("Place sensor flat and stationary...\n");
 *     sleep_ms(2000);
 *     jy901_start_acc_calibration(&imu);
 *     sleep_ms(5000);
 *     jy901_stop_calibration(&imu);
 *     jy901_save(&imu);
 *     printf("Acc calibration done.\n");
 * }
 *
 * void do_mag_calibration(void) {
 *     printf("Starting mag calibration - rotate sensor through all orientations...\n");
 *     jy901_start_mag_calibration(&imu);
 *     sleep_ms(15000);  // Give time to rotate
 *     jy901_stop_calibration(&imu);
 *     jy901_save(&imu);
 *     printf("Mag calibration done.\n");
 * }
 */
