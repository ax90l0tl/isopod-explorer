/*
 * Minimal Raspberry Pi Pico example for the LPS C driver.
 *
 * Default wiring (I2C0):
 *   Sensor SDA -> GP4 (Pico pin 6)
 *   Sensor SCL -> GP5 (Pico pin 7)
 *   Sensor VIN -> 3.3 V
 *   Sensor GND -> GND
 *
 * Add lps.c to your CMake target and link hardware_i2c.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lps.h"

#define I2C_PORT        i2c0
#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define I2C_BAUDRATE    400000

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    lps_t ps;
    lps_init_struct(&ps, I2C_PORT);

    if (!lps_init(&ps, LPS_DEVICE_AUTO, LPS_SA0_AUTO)) {
        printf("Failed to detect LPS pressure sensor\n");
        while (true) {
            tight_loop_contents();
        }
    }

    printf("Detected device type %d at address 0x%02X\n",
           (int)ps.device, ps.address);

    lps_enable_default(&ps);
    sleep_ms(100);

    while (true) {
        if (lps_read_data(&ps)) {
            printf("p: %.2f mbar    a: %.2f m    t: %.2f deg C\n",
                   ps.pressure, ps.altitude, ps.temperature);
        } else {
            printf("Read failed\n");
        }
        sleep_ms(200);
    }
}
