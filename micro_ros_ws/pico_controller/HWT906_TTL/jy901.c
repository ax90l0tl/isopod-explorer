#include "jy901.h"
#include <string.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

void jy901_init(jy901_t *dev)
{
    if (!dev) return;
    memset(dev, 0, sizeof(jy901_t));
    dev->i2c_addr = 0x50;
    dev->rx_cnt = 0;
    dev->i2c = NULL;
}

bool jy901_init_i2c(jy901_t *dev, i2c_inst_t *i2c, uint8_t addr)
{
    if (!dev) return false;
    jy901_init(dev);
    dev->i2c = i2c;
    dev->i2c_addr = addr;
    return true;
}

void jy901_cope_serial_data(jy901_t *dev, uint8_t data)
{
    if (!dev) return;

    dev->rx_buffer[dev->rx_cnt++] = data;

    if (dev->rx_buffer[0] != 0x55) {
        dev->rx_cnt = 0;
        return;
    }

    if (dev->rx_cnt < 11) {
        return;
    }

    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += dev->rx_buffer[i];
    }
    if (sum != dev->rx_buffer[10]) {
        dev->rx_cnt = 0;
        return;
    }

    const uint8_t *payload = &dev->rx_buffer[2];
    switch (dev->rx_buffer[1]) {
        case 0x50: memcpy(&dev->time,    payload, 8); break;
        case 0x51: memcpy(&dev->acc,     payload, 8); break;
        case 0x52: memcpy(&dev->gyro,    payload, 8); break;
        case 0x53: memcpy(&dev->angle,   payload, 8); break;
        case 0x54: memcpy(&dev->mag,     payload, 8); break;
        case 0x55: memcpy(&dev->dstatus, payload, 8); break;
        case 0x56: memcpy(&dev->press,   payload, 8); break;
        case 0x57: memcpy(&dev->lonlat,  payload, 8); break;
        case 0x58: memcpy(&dev->gpsv,    payload, 8); break;
        case 0x59: memcpy(&dev->quat,    payload, 8); break;
        case 0x5A: memcpy(&dev->sn,      payload, 8); break;
        default: break;
    }

    dev->rx_cnt = 0;
}

int jy901_read_registers(jy901_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    if (!dev || !dev->i2c || !buf || len == 0) return -1;

    int ret = i2c_write_blocking(dev->i2c, dev->i2c_addr, &reg, 1, true);
    if (ret < 0) return ret;

    ret = i2c_read_blocking(dev->i2c, dev->i2c_addr, buf, len, false);
    return ret;
}

int jy901_write_registers(jy901_t *dev, uint8_t reg, const uint8_t *data, size_t len)
{
    if (!dev || !dev->i2c || !data || len == 0) return -1;

    uint8_t buf[1 + len];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    return i2c_write_blocking(dev->i2c, dev->i2c_addr, buf, 1 + len, false);
}

int16_t jy901_read_word(jy901_t *dev, uint8_t reg)
{
    uint8_t buf[2] = {0};
    if (jy901_read_registers(dev, reg, buf, 2) == 2) {
        return (int16_t)(buf[0] | (buf[1] << 8));
    }
    return 0;
}

void jy901_write_word(jy901_t *dev, uint8_t reg, int16_t value)
{
    uint8_t buf[2];
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
    jy901_write_registers(dev, reg, buf, 2);
}

void jy901_get_time(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_YYMM, (uint8_t *)&dev->time, sizeof(dev->time));
}

void jy901_get_acc(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_AX, (uint8_t *)&dev->acc, 6);
}

void jy901_get_gyro(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_GX, (uint8_t *)&dev->gyro, 6);
}

void jy901_get_angle(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_ROLL, (uint8_t *)&dev->angle, 6);
}

void jy901_get_mag(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_HX, (uint8_t *)&dev->mag, 6);
}

void jy901_get_press(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_PRESSUREL, (uint8_t *)&dev->press, sizeof(dev->press));
}

void jy901_get_dstatus(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_D0STATUS, (uint8_t *)&dev->dstatus, sizeof(dev->dstatus));
}

void jy901_get_lonlat(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_LONL, (uint8_t *)&dev->lonlat, sizeof(dev->lonlat));
}

void jy901_get_gpsv(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_GPSHEIGHT, (uint8_t *)&dev->gpsv, sizeof(dev->gpsv));
}

void jy901_get_quaternion(jy901_t *dev)
{
    jy901_read_registers(dev, JY901_Q0, (uint8_t *)&dev->quat, sizeof(dev->quat));
}

// --- Configuration ---

void jy901_save(jy901_t *dev)
{
    jy901_write_word(dev, JY901_SAVE, 0x0000);
    sleep_ms(100);  // Give the sensor time to write flash
}

void jy901_factory_reset(jy901_t *dev)
{
    jy901_write_word(dev, JY901_SAVE, 0x0001);
    sleep_ms(100);
}

void jy901_set_output_rate(jy901_t *dev, uint16_t rate_code)
{
    jy901_write_word(dev, JY901_RRATE, rate_code);
}

void jy901_set_baudrate(jy901_t *dev, uint16_t baud_code)
{
    jy901_write_word(dev, JY901_BAUD, baud_code);
}

void jy901_set_output_content(jy901_t *dev, uint16_t rsw_bits)
{
    jy901_write_word(dev, JY901_RSW, rsw_bits);
}

void jy901_enable_quaternion(jy901_t *dev)
{
    uint16_t rsw = jy901_read_word(dev, JY901_RSW);
    rsw |= JY901_RSW_QUATER;
    jy901_write_word(dev, JY901_RSW, rsw);
}

// --- Calibration ---

void jy901_start_acc_calibration(jy901_t *dev)
{
    // Place sensor flat and stationary before calling
    jy901_write_word(dev, JY901_CALSW, 0x0001);
}

void jy901_start_mag_calibration(jy901_t *dev)
{
    // Start magnetic field calibration (spherical fitting)
    jy901_write_word(dev, JY901_CALSW, 0x0007);
}

void jy901_stop_calibration(jy901_t *dev)
{
    jy901_write_word(dev, JY901_CALSW, 0x0000);
}

void jy901_reset_yaw(jy901_t *dev)
{
    jy901_write_word(dev, JY901_CALSW, 0x0004);
}
