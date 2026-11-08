#ifndef JY901_H
#define JY901_H

#include <stdint.h>
#include <stddef.h>
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register addresses
#define JY901_SAVE          0x00
#define JY901_CALSW         0x01
#define JY901_RSW           0x02
#define JY901_RRATE         0x03
#define JY901_BAUD          0x04
#define JY901_AXOFFSET      0x05
#define JY901_AYOFFSET      0x06
#define JY901_AZOFFSET      0x07
#define JY901_GXOFFSET      0x08
#define JY901_GYOFFSET      0x09
#define JY901_GZOFFSET      0x0A
#define JY901_HXOFFSET      0x0B
#define JY901_HYOFFSET      0x0C
#define JY901_HZOFFSET      0x0D
#define JY901_IICADDR       0x1A
#define JY901_LEDOFF        0x1B

#define JY901_YYMM          0x30
#define JY901_DDHH          0x31
#define JY901_MMSS          0x32
#define JY901_MS            0x33
#define JY901_AX            0x34
#define JY901_AY            0x35
#define JY901_AZ            0x36
#define JY901_GX            0x37
#define JY901_GY            0x38
#define JY901_GZ            0x39
#define JY901_HX            0x3A
#define JY901_HY            0x3B
#define JY901_HZ            0x3C
#define JY901_ROLL          0x3D
#define JY901_PITCH         0x3E
#define JY901_YAW           0x3F
#define JY901_TEMP          0x40
#define JY901_D0STATUS      0x41
#define JY901_PRESSUREL     0x45
#define JY901_LONL          0x49
#define JY901_GPSHEIGHT     0x4D
#define JY901_Q0            0x51
#define JY901_Q1            0x52
#define JY901_Q2            0x53
#define JY901_Q3            0x54

// Output rate codes (RRATE register)
#define JY901_RATE_0_2HZ    0x01
#define JY901_RATE_0_5HZ    0x02
#define JY901_RATE_1HZ      0x03
#define JY901_RATE_2HZ      0x04
#define JY901_RATE_5HZ      0x05
#define JY901_RATE_10HZ     0x06
#define JY901_RATE_20HZ     0x07
#define JY901_RATE_50HZ     0x08
#define JY901_RATE_100HZ    0x09
#define JY901_RATE_200HZ    0x0B
#define JY901_RATE_500HZ    0x0C
#define JY901_RATE_1000HZ   0x0D

// Baud rate codes (BAUD register)
#define JY901_BAUD_4800     0x01
#define JY901_BAUD_9600     0x02
#define JY901_BAUD_19200    0x03
#define JY901_BAUD_38400    0x04
#define JY901_BAUD_57600    0x05
#define JY901_BAUD_115200   0x06
#define JY901_BAUD_230400   0x07
#define JY901_BAUD_460800   0x08
#define JY901_BAUD_921600   0x09

// RSW bits (output content)
#define JY901_RSW_TIME      (1 << 0)
#define JY901_RSW_ACC       (1 << 1)
#define JY901_RSW_GYRO      (1 << 2)
#define JY901_RSW_ANGLE     (1 << 3)
#define JY901_RSW_MAG       (1 << 4)
#define JY901_RSW_PORT      (1 << 5)
#define JY901_RSW_PRESS     (1 << 6)
#define JY901_RSW_GPS       (1 << 7)
#define JY901_RSW_VELOCITY  (1 << 8)
#define JY901_RSW_QUATER    (1 << 9)
#define JY901_RSW_GSA       (1 << 10)

// Data structures
typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} jy901_time_t;

typedef struct {
    int16_t a[3];
    int16_t temp;
} jy901_acc_t;

typedef struct {
    int16_t w[3];
    int16_t temp;
} jy901_gyro_t;

typedef struct {
    int16_t angle[3];
    int16_t temp;
} jy901_angle_t;

typedef struct {
    int16_t h[3];
    int16_t temp;
} jy901_mag_t;

typedef struct {
    int16_t status[4];
} jy901_dstatus_t;

typedef struct {
    int32_t pressure;
    int32_t altitude;
} jy901_press_t;

typedef struct {
    int32_t lon;
    int32_t lat;
} jy901_lonlat_t;

typedef struct {
    int16_t gps_height;
    int16_t gps_yaw;
    int32_t gps_velocity;
} jy901_gpsv_t;

typedef struct {
    int16_t q0, q1, q2, q3;
} jy901_quat_t;

typedef struct {
    int16_t sv_num;
    int16_t pdop;
    int16_t hdop;
    int16_t vdop;
} jy901_sn_t;

typedef struct {
    jy901_time_t time;
    jy901_acc_t acc;
    jy901_gyro_t gyro;
    jy901_angle_t angle;
    jy901_mag_t mag;
    jy901_dstatus_t dstatus;
    jy901_press_t press;
    jy901_lonlat_t lonlat;
    jy901_gpsv_t gpsv;
    jy901_quat_t quat;
    jy901_sn_t sn;

    // Internal parser state (for serial mode)
    uint8_t rx_buffer[11];
    uint8_t rx_cnt;

    // I2C state
    i2c_inst_t *i2c;
    uint8_t i2c_addr;  // default 0x50
} jy901_t;

/**
 * Initialize a JY901 device structure.
 */
void jy901_init(jy901_t *dev);

/**
 * Initialize for I2C communication.
 */
bool jy901_init_i2c(jy901_t *dev, i2c_inst_t *i2c, uint8_t addr);

/**
 * Feed one byte from the UART into the parser (serial mode).
 */
void jy901_cope_serial_data(jy901_t *dev, uint8_t data);

/**
 * Read registers over I2C.
 */
int jy901_read_registers(jy901_t *dev, uint8_t reg, uint8_t *buf, size_t len);

/**
 * Write registers over I2C.
 */
int jy901_write_registers(jy901_t *dev, uint8_t reg, const uint8_t *data, size_t len);

/**
 * Read a 16-bit word (little-endian) from a register.
 */
int16_t jy901_read_word(jy901_t *dev, uint8_t reg);

/**
 * Write a 16-bit word to a register.
 */
void jy901_write_word(jy901_t *dev, uint8_t reg, int16_t value);

// High-level read functions (I2C)
void jy901_get_time(jy901_t *dev);
void jy901_get_acc(jy901_t *dev);
void jy901_get_gyro(jy901_t *dev);
void jy901_get_angle(jy901_t *dev);
void jy901_get_mag(jy901_t *dev);
void jy901_get_press(jy901_t *dev);
void jy901_get_dstatus(jy901_t *dev);
void jy901_get_lonlat(jy901_t *dev);
void jy901_get_gpsv(jy901_t *dev);
void jy901_get_quaternion(jy901_t *dev);

// --- Configuration / Settings ---

/**
 * Save current configuration to non-volatile memory.
 */
void jy901_save(jy901_t *dev);

/**
 * Restore factory defaults.
 */
void jy901_factory_reset(jy901_t *dev);

/**
 * Set output data rate.
 * @param rate_code One of JY901_RATE_* constants
 */
void jy901_set_output_rate(jy901_t *dev, uint16_t rate_code);

/**
 * Set serial baud rate (affects UART mode).
 * @param baud_code One of JY901_BAUD_* constants
 */
void jy901_set_baudrate(jy901_t *dev, uint16_t baud_code);

/**
 * Configure which data packets are output (RSW register).
 * Use JY901_RSW_* bitmasks ORed together.
 */
void jy901_set_output_content(jy901_t *dev, uint16_t rsw_bits);

/**
 * Enable quaternion output (sets the QUATER bit in RSW).
 */
void jy901_enable_quaternion(jy901_t *dev);

// --- Calibration helpers ---

/**
 * Start accelerometer calibration.
 * Place the sensor flat and stationary (Z-axis up) before calling.
 * Wait ~5 seconds after calling, then call jy901_stop_calibration() and jy901_save().
 */
void jy901_start_acc_calibration(jy901_t *dev);

/**
 * Start magnetic field calibration (spherical fitting).
 * Rotate the sensor slowly through all orientations (figure-8 or all axes) after calling.
 * Then call jy901_stop_calibration() and jy901_save().
 */
void jy901_start_mag_calibration(jy901_t *dev);

/**
 * Exit calibration mode (write 0x00 to CALSW).
 * Call this after acc or mag calibration, then save.
 */
void jy901_stop_calibration(jy901_t *dev);

/**
 * Reset yaw (heading) to zero.
 */
void jy901_reset_yaw(jy901_t *dev);

// --- Conversion helpers ---

static inline float jy901_acc_to_g(int16_t raw) {
    return (float)raw / 32768.0f * 16.0f;
}

static inline float jy901_gyro_to_dps(int16_t raw) {
    return (float)raw / 32768.0f * 2000.0f;
}

static inline float jy901_angle_to_deg(int16_t raw) {
    return (float)raw / 32768.0f * 180.0f;
}

/**
 * Convert raw quaternion components to unit quaternion (range -1.0 to +1.0).
 */
static inline void jy901_quat_to_float(const jy901_quat_t *q, float *out_q0, float *out_q1, float *out_q2, float *out_q3) {
    *out_q0 = (float)q->q0 / 32768.0f;
    *out_q1 = (float)q->q1 / 32768.0f;
    *out_q2 = (float)q->q2 / 32768.0f;
    *out_q3 = (float)q->q3 / 32768.0f;
}

#ifdef __cplusplus
}
#endif

#endif // JY901_H
