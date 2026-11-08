#ifndef LPS_H
#define LPS_H

#include <stdint.h>
#include "hardware/i2c.h"

/* 7-bit I2C addresses (SA0 selects the LSB) */
#define LPS_SA0_LOW_ADDRESS 0x5C  /* 0b1011100 */
#define LPS_SA0_HIGH_ADDRESS 0x5D /* 0b1011101 */

#define LPS_TEST_REG_NACK (-1)

#define LPS331AP_WHO_ID 0xBB
#define LPS25H_WHO_ID 0xBD
#define LPS22DF_WHO_ID 0xB4

typedef enum
{
    LPS_DEVICE_331AP = 0,
    LPS_DEVICE_25H,
    LPS_DEVICE_22DF,
    LPS_DEVICE_AUTO
} lps_device_t;

typedef enum
{
    LPS_SA0_LOW = 0,
    LPS_SA0_HIGH,
    LPS_SA0_AUTO
} lps_sa0_t;

/*
 * Register addresses.
 * Negative values are dummy addresses that are translated per device type
 * after init().
 */
typedef enum
{
    LPS_REF_P_XL = 0x08,
    LPS_IF_CTRL = 0x0E, /* 22DF */
    LPS_WHO_AM_I = 0x0F,
    LPS_RES_CONF = 0x10,
    LPS_FIFO_WTM = 0x15,        /* 22DF */
    LPS_I3C_IF_CTRL_ADD = 0x19, /* 22DF */
    LPS_FIFO_STATUS1 = 0x25,    /* 22DF */
    LPS_FIFO_STATUS2 = 0x26,    /* 22DF */
    LPS_STATUS_REG = 0x27,
    LPS_PRESS_OUT_XL = 0x28,
    LPS_PRESS_OUT_L = 0x29,
    LPS_PRESS_OUT_H = 0x2A,
    LPS_TEMP_OUT_L = 0x2B,
    LPS_TEMP_OUT_H = 0x2C,
    LPS_FIFO_STATUS = 0x2F,            /* 25H */
    LPS_AMP_CTRL = 0x30,               /* 331AP */
    LPS_DELTA_PRESS_XL = 0x3C,         /* 331AP */
    LPS_DELTA_PRESS_L = 0x3D,          /* 331AP */
    LPS_DELTA_PRESS_H = 0x3E,          /* 331AP */
    LPS_FIFO_DATA_OUT_PRESS_XL = 0x78, /* 22DF */
    LPS_FIFO_DATA_OUT_PRESS_L = 0x79,  /* 22DF */
    LPS_FIFO_DATA_OUT_PRESS_H = 0x7A,  /* 22DF */

    /* Dummy addresses (translated after init) */
    LPS_REF_P_L = -1,
    LPS_REF_P_H = -2,
    LPS_CTRL_REG1 = -3,
    LPS_CTRL_REG2 = -4,
    LPS_CTRL_REG3 = -5,
    LPS_CTRL_REG4 = -6,
    LPS_INTERRUPT_CFG = -7,
    LPS_INT_SOURCE = -8,
    LPS_FIFO_CTRL = -9,
    LPS_THS_P_L = -10,
    LPS_THS_P_H = -11,
    LPS_RPDS_L = -12,
    LPS_RPDS_H = -13,

    /* LPS331AP-specific */
    LPS331AP_REF_P_L = 0x09,
    LPS331AP_REF_P_H = 0x0A,
    LPS331AP_CTRL_REG1 = 0x20,
    LPS331AP_CTRL_REG2 = 0x21,
    LPS331AP_CTRL_REG3 = 0x22,
    LPS331AP_INTERRUPT_CFG = 0x23,
    LPS331AP_INT_SOURCE = 0x24,
    LPS331AP_THS_P_L = 0x25,
    LPS331AP_THS_P_H = 0x26,

    /* LPS25H-specific */
    LPS25H_REF_P_L = 0x09,
    LPS25H_REF_P_H = 0x0A,
    LPS25H_CTRL_REG1 = 0x20,
    LPS25H_CTRL_REG2 = 0x21,
    LPS25H_CTRL_REG3 = 0x22,
    LPS25H_CTRL_REG4 = 0x23,
    LPS25H_INTERRUPT_CFG = 0x24,
    LPS25H_INT_SOURCE = 0x25,
    LPS25H_FIFO_CTRL = 0x2E,
    LPS25H_THS_P_L = 0x30,
    LPS25H_THS_P_H = 0x31,
    LPS25H_RPDS_L = 0x39,
    LPS25H_RPDS_H = 0x3A,

    /* LPS22DF-specific */
    LPS22DF_INTERRUPT_CFG = 0x0B,
    LPS22DF_THS_P_L = 0x0C,
    LPS22DF_THS_P_H = 0x0D,
    LPS22DF_CTRL_REG1 = 0x10,
    LPS22DF_CTRL_REG2 = 0x11,
    LPS22DF_CTRL_REG3 = 0x12,
    LPS22DF_CTRL_REG4 = 0x13,
    LPS22DF_FIFO_CTRL = 0x14,
    LPS22DF_REF_P_L = 0x16,
    LPS22DF_REF_P_H = 0x17,
    LPS22DF_RPDS_L = 0x1A,
    LPS22DF_RPDS_H = 0x1B,
    LPS22DF_INT_SOURCE = 0x24
} lps_reg_t;

#define LPS_DUMMY_REG_COUNT 13

typedef struct
{
    i2c_inst_t *i2c; /* i2c0 or i2c1, already initialized by caller */
    uint8_t address; /* 7-bit I2C address */
    lps_device_t device;
    int16_t translated_regs[LPS_DUMMY_REG_COUNT + 1];

    float temperature; /* last cached reading, deg C */
    float pressure;    /* last cached reading, mbar */
    float altitude;    /* last cached reading, meters */
} lps_t;

/* Initialize the driver object. Does not talk to the sensor yet.
 * i2c must already be configured (pins + baudrate) by the caller. */
void lps_init_struct(lps_t *ps, i2c_inst_t *i2c);

/* Detect device type and I2C address. Returns true on success. */
bool lps_init(lps_t *ps, lps_device_t device, lps_sa0_t sa0);

/* Power on and start continuous conversion at the library default ODR. */
void lps_enable_default(lps_t *ps);

bool lps_write_reg(lps_t *ps, int reg, uint8_t value);
bool lps_read_reg(lps_t *ps, int reg, uint8_t *value);

int32_t lps_read_pressure_raw(lps_t *ps);
int16_t lps_read_temperature_raw(lps_t *ps);

float lps_read_pressure_millibars(lps_t *ps);
float lps_read_pressure_inches_hg(lps_t *ps);
float lps_read_temperature_c(lps_t *ps);
float lps_read_temperature_f(lps_t *ps);

/* Read pressure + temperature and update the cached fields (and altitude). */
bool lps_read_data(lps_t *ps);

float lps_pressure_to_altitude_meters(float pressure_mbar, float altimeter_setting_mbar);
float lps_pressure_to_altitude_feet(float pressure_inHg, float altimeter_setting_inHg);

#endif /* LPS_H */
