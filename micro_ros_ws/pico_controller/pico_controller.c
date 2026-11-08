#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "lps331ap_i2c/LPS.h"
#include "HWT906_TTL/jy901.h"
#include "micro_ros_raspberrypi_pico_sdk/pico_uart_transports.h"
// // SPI Defines
// // We are going to use SPI 0, and allocate it to the following GPIO pins
// // Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

// // I2C defines
// // This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// // Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21

// // Data will be copied from src to dst
// const char src[] = "Hello, world! (from DMA)";
// char dst[count_of(src)];

// int64_t alarm_callback(alarm_id_t id, void *user_data) {
//     // Put your timeout handler code in here
//     return 0;
// }

// // UART defines
// // By default the stdout UART is `uart0`, so we will use the second one
// #define UART_ID uart1
// #define BAUD_RATE 115200

// // Use pins 4 and 5 for UART1
// // Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// #define UART_TX_PIN 4
// #define UART_RX_PIN 5

lps_t lps;
jy901_t imu;


int reserved_addr(uint8_t addr)
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void i2c_scan(i2c_inst_t *i2c)
{
    printf("\n     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr)
    {
        if (addr % 16 == 0)
        {
            printf("%02x: ", addr);
        }

        // Skip reserved addresses
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
        {
            ret = PICO_ERROR_GENERIC;
        }
        else
        {
            // Attempt a 1-byte read from the address
            ret = i2c_read_blocking(i2c, addr, &rxdata, 1, false);
        }

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
}

int main()
{
    stdio_init_all();

    //     // SPI initialisation. This example will use SPI at 1MHz.
    //     spi_init(SPI_PORT, 1000*1000);
    //     gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    //     gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    //     gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    //     gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    //     // Chip select is active-low, so we'll initialise it to a driven-high state
    //     gpio_set_dir(PIN_CS, GPIO_OUT);
    //     gpio_put(PIN_CS, 1);
    //     // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    //     // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    lps_init_struct(&lps, I2C_PORT);
    jy901_init(&imu);

    while (!jy901_init_i2c(&imu, I2C_PORT, 0x50))
    {
        i2c_scan(I2C_PORT);
        sleep_ms(3000);
    }

    while (!lps_init(&lps, LPS_DEVICE_AUTO, LPS_SA0_AUTO))
    {
        i2c_scan(I2C_PORT);
        sleep_ms(3000);
    }

    //     // Get a free channel, panic() if there are none
    //     int chan = dma_claim_unused_channel(true);

    //     // 8 bit transfers. Both read and write address increment after each
    //     // transfer (each pointing to a location in src or dst respectively).
    //     // No DREQ is selected, so the DMA transfers as fast as it can.

    //     dma_channel_config c = dma_channel_get_default_config(chan);
    //     channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    //     channel_config_set_read_increment(&c, true);
    //     channel_config_set_write_increment(&c, true);

    //     dma_channel_configure(
    //         chan,          // Channel to be configured
    //         &c,            // The configuration we just created
    //         dst,           // The initial write address
    //         src,           // The initial read address
    //         count_of(src), // Number of transfers; in this case each is 1 byte.
    //         true           // Start immediately.
    //     );

    //     // We could choose to go and do something else whilst the DMA is doing its
    //     // thing. In this case the processor has nothing else to do, so we just
    //     // wait for the DMA to finish.
    //     dma_channel_wait_for_finish_blocking(chan);

    //     // The DMA has now copied our text from the transmit buffer (src) to the
    //     // receive buffer (dst), so we can print it out from there.
    //     puts(dst);

    //     // PIO Blinking example
    //     PIO pio = pio0;

    //     // Interpolator example code
    //     interp_config cfg = interp_default_config();
    //     // Now use the various interpolator library functions for your use case
    //     // e.g. interp_config_clamp(&cfg, true);
    //     //      interp_config_shift(&cfg, 2);
    //     // Then set the config
    //     interp_set_config(interp0, 0, &cfg);
    //     // For examples of interpolator use see https://github.com/raspberrypi/pico-examples/tree/master/interp

    //     // Timer example code - This example fires off the callback after 2000ms
    //     add_alarm_in_ms(2000, alarm_callback, NULL, false);
    //     // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    //     // Watchdog example code
    //     if (watchdog_caused_reboot()) {
    //         printf("Rebooted by Watchdog!\n");
    //         // Whatever action you may take if a watchdog caused a reboot
    //     }

    //     // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
    //     // second arg is pause on debug which means the watchdog will pause when stepping through code
    //     watchdog_enable(100, 1);

    //     // You need to call this function at least more often than the 100ms in the enable call to prevent a reboot
    //     watchdog_update();

    //     printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    //     printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    //     // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

    //     // Set up our UART
    //     uart_init(UART_ID, BAUD_RATE);
    //     // Set the TX and RX pins by using the function select on the GPIO
    //     // Set datasheet for more information on function select
    //     gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    //     gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    //     // Use some the various UART functions to send out data
    //     // In a default system, printf will also output via the default UART

    //     // Send out a string, with CR/LF conversions
    //     uart_puts(UART_ID, " Hello, UART!\n");

    //     // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart
    jy901_set_output_rate(&imu, JY901_RATE_1000HZ);
    jy901_enable_quaternion(&imu);
    jy901_save(&imu);
    lps_enable_default(&lps);

    while (true)
    {
                // Read data via I2C
        jy901_get_acc(&imu);
        jy901_get_gyro(&imu);
        jy901_get_quaternion(&imu);
        jy901_get_mag(&imu);

        printf("Acc: %.3f %.3f %.3f g\n",
               jy901_acc_to_g(imu.acc.a[0]),
               jy901_acc_to_g(imu.acc.a[1]),
               jy901_acc_to_g(imu.acc.a[2]));

        // printf("Gyro: %.2f %.2f %.2f dps\n",
        //        jy901_gyro_to_dps(imu.gyro.w[0]),
        //        jy901_gyro_to_dps(imu.gyro.w[1]),
        //        jy901_gyro_to_dps(imu.gyro.w[2]));

        float q0, q1, q2, q3;
        jy901_quat_to_float(&imu.quat, &q0, &q1, &q2, &q3);
        // printf("Quat: %.4f %.4f %.4f %.4f\n", q0, q1, q2, q3);

        // printf("Mag: %d %d %d\n", imu.mag.h[0], imu.mag.h[1], imu.mag.h[2]);
        // printf("----\n");

        sleep_ms(1);
        // i2c_scan(I2C_PORT);
        // sleep_ms(3000);
        // if (lps_read_data(&lps))
        // {
            // printf("p: %.2f mbar    a: %.2f m    t: %.2f deg C\n",
                //    lps.pressure, lps.altitude, lps.temperature);
        // }
    }
}
