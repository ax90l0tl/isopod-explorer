#include "LPS.h"

#include <math.h>
#include "pico/stdlib.h"

static int resolve_reg(lps_t *ps, int reg)
{
    if (reg < 0) {
        int idx = -reg;
        if (idx < 1 || idx > LPS_DUMMY_REG_COUNT) {
            return -1;
        }
        return ps->translated_regs[idx];
    }
    return reg;
}

static bool i2c_write_bytes(lps_t *ps, const uint8_t *data, size_t len)
{
    int n = i2c_write_blocking(ps->i2c, ps->address, data, len, false);
    return n == (int)len;
}

static bool i2c_write_then_read(lps_t *ps, uint8_t reg, uint8_t *buf, size_t len)
{
    int n = i2c_write_blocking(ps->i2c, ps->address, &reg, 1, true);
    if (n != 1) {
        return false;
    }
    n = i2c_read_blocking(ps->i2c, ps->address, buf, len, false);
    return n == (int)len;
}

static int test_who_am_i(lps_t *ps, uint8_t address)
{
    uint8_t reg = LPS_WHO_AM_I;
    uint8_t id = 0;
    uint8_t saved = ps->address;

    ps->address = address;
    int n = i2c_write_blocking(ps->i2c, address, &reg, 1, true);
    if (n != 1) {
        ps->address = saved;
        return LPS_TEST_REG_NACK;
    }
    n = i2c_read_blocking(ps->i2c, address, &id, 1, false);
    ps->address = saved;
    if (n != 1) {
        return LPS_TEST_REG_NACK;
    }
    return (int)id;
}

static bool detect_device(lps_t *ps, lps_device_t device)
{
    int id = test_who_am_i(ps, ps->address);

    if ((device == LPS_DEVICE_AUTO || device == LPS_DEVICE_22DF) && id == LPS22DF_WHO_ID) {
        ps->device = LPS_DEVICE_22DF;
        return true;
    }
    if ((device == LPS_DEVICE_AUTO || device == LPS_DEVICE_25H) && id == LPS25H_WHO_ID) {
        ps->device = LPS_DEVICE_25H;
        return true;
    }
    if ((device == LPS_DEVICE_AUTO || device == LPS_DEVICE_331AP) && id == LPS331AP_WHO_ID) {
        ps->device = LPS_DEVICE_331AP;
        return true;
    }
    return false;
}

static bool detect_device_and_address(lps_t *ps, lps_device_t device, lps_sa0_t sa0)
{
    if (sa0 == LPS_SA0_AUTO || sa0 == LPS_SA0_HIGH) {
        ps->address = LPS_SA0_HIGH_ADDRESS;
        if (detect_device(ps, device)) {
            return true;
        }
    }
    if (sa0 == LPS_SA0_AUTO || sa0 == LPS_SA0_LOW) {
        ps->address = LPS_SA0_LOW_ADDRESS;
        if (detect_device(ps, device)) {
            return true;
        }
    }
    return false;
}

void lps_init_struct(lps_t *ps, i2c_inst_t *i2c)
{
    ps->i2c = i2c;
    ps->device = LPS_DEVICE_AUTO;
    /* Pololu carriers pull SA0 high by default */
    ps->address = LPS_SA0_HIGH_ADDRESS;
    ps->temperature = 0.0f;
    ps->pressure = 0.0f;
    ps->altitude = 0.0f;
    for (int i = 0; i <= LPS_DUMMY_REG_COUNT; i++) {
        ps->translated_regs[i] = 0;
    }
}

bool lps_init(lps_t *ps, lps_device_t device, lps_sa0_t sa0)
{
    if (!detect_device_and_address(ps, device, sa0)) {
        return false;
    }

    switch (ps->device) {
    case LPS_DEVICE_25H:
        ps->translated_regs[-LPS_REF_P_L]       = LPS25H_REF_P_L;
        ps->translated_regs[-LPS_REF_P_H]       = LPS25H_REF_P_H;
        ps->translated_regs[-LPS_CTRL_REG1]     = LPS25H_CTRL_REG1;
        ps->translated_regs[-LPS_CTRL_REG2]     = LPS25H_CTRL_REG2;
        ps->translated_regs[-LPS_CTRL_REG3]     = LPS25H_CTRL_REG3;
        ps->translated_regs[-LPS_CTRL_REG4]     = LPS25H_CTRL_REG4;
        ps->translated_regs[-LPS_INTERRUPT_CFG] = LPS25H_INTERRUPT_CFG;
        ps->translated_regs[-LPS_INT_SOURCE]    = LPS25H_INT_SOURCE;
        ps->translated_regs[-LPS_FIFO_CTRL]     = LPS25H_FIFO_CTRL;
        ps->translated_regs[-LPS_THS_P_L]       = LPS25H_THS_P_L;
        ps->translated_regs[-LPS_THS_P_H]       = LPS25H_THS_P_H;
        ps->translated_regs[-LPS_RPDS_L]        = LPS25H_RPDS_L;
        ps->translated_regs[-LPS_RPDS_H]        = LPS25H_RPDS_H;
        return true;

    case LPS_DEVICE_331AP:
        ps->translated_regs[-LPS_REF_P_L]       = LPS331AP_REF_P_L;
        ps->translated_regs[-LPS_REF_P_H]       = LPS331AP_REF_P_H;
        ps->translated_regs[-LPS_CTRL_REG1]     = LPS331AP_CTRL_REG1;
        ps->translated_regs[-LPS_CTRL_REG2]     = LPS331AP_CTRL_REG2;
        ps->translated_regs[-LPS_CTRL_REG3]     = LPS331AP_CTRL_REG3;
        ps->translated_regs[-LPS_INTERRUPT_CFG] = LPS331AP_INTERRUPT_CFG;
        ps->translated_regs[-LPS_INT_SOURCE]    = LPS331AP_INT_SOURCE;
        ps->translated_regs[-LPS_THS_P_L]       = LPS331AP_THS_P_L;
        ps->translated_regs[-LPS_THS_P_H]       = LPS331AP_THS_P_H;
        return true;

    case LPS_DEVICE_22DF:
        ps->translated_regs[-LPS_REF_P_L]       = LPS22DF_REF_P_L;
        ps->translated_regs[-LPS_REF_P_H]       = LPS22DF_REF_P_H;
        ps->translated_regs[-LPS_CTRL_REG1]     = LPS22DF_CTRL_REG1;
        ps->translated_regs[-LPS_CTRL_REG2]     = LPS22DF_CTRL_REG2;
        ps->translated_regs[-LPS_CTRL_REG3]     = LPS22DF_CTRL_REG3;
        ps->translated_regs[-LPS_CTRL_REG4]     = LPS22DF_CTRL_REG4;
        ps->translated_regs[-LPS_INTERRUPT_CFG] = LPS22DF_INTERRUPT_CFG;
        ps->translated_regs[-LPS_INT_SOURCE]    = LPS22DF_INT_SOURCE;
        ps->translated_regs[-LPS_FIFO_CTRL]     = LPS22DF_FIFO_CTRL;
        ps->translated_regs[-LPS_THS_P_L]       = LPS22DF_THS_P_L;
        ps->translated_regs[-LPS_THS_P_H]       = LPS22DF_THS_P_H;
        ps->translated_regs[-LPS_RPDS_L]        = LPS22DF_RPDS_L;
        ps->translated_regs[-LPS_RPDS_H]        = LPS22DF_RPDS_H;
        return true;

    default:
        return false;
    }
}

void lps_enable_default(lps_t *ps)
{
    if (ps->device == LPS_DEVICE_22DF) {
        /* ODR = 0011 (10 Hz); AVG = 000 (4 averages) */
        lps_write_reg(ps, LPS_CTRL_REG1, 0x18);
        /* IF_ADD_INC = 1 (auto-increment) */
        lps_write_reg(ps, LPS_CTRL_REG3, 0x01);
    } else if (ps->device == LPS_DEVICE_25H) {
        /* PD = 1; ODR = 011 (12.5 Hz) */
        lps_write_reg(ps, LPS_CTRL_REG1, 0xB0);
    } else if (ps->device == LPS_DEVICE_331AP) {
        /* PD = 1; ODR = 110 (12.5 Hz) */
        lps_write_reg(ps, LPS_CTRL_REG1, 0xE0);
    }
}

bool lps_write_reg(lps_t *ps, int reg, uint8_t value)
{
    int resolved = resolve_reg(ps, reg);
    if (resolved < 0) {
        return false;
    }
    uint8_t buf[2] = { (uint8_t)resolved, value };
    return i2c_write_bytes(ps, buf, 2);
}

bool lps_read_reg(lps_t *ps, int reg, uint8_t *value)
{
    int resolved = resolve_reg(ps, reg);
    if (resolved < 0 || value == NULL) {
        return false;
    }
    return i2c_write_then_read(ps, (uint8_t)resolved, value, 1);
}

int32_t lps_read_pressure_raw(lps_t *ps)
{
    uint8_t start = LPS_PRESS_OUT_XL;
    uint8_t raw[3];

    /* LPS331AP / LPS25H need the MSB set to enable address auto-increment */
    if (ps->device == LPS_DEVICE_25H || ps->device == LPS_DEVICE_331AP) {
        start = (uint8_t)(LPS_PRESS_OUT_XL | (1u << 7));
    }

    if (!i2c_write_then_read(ps, start, raw, 3)) {
        return 0;
    }

    uint8_t pxl = raw[0];
    uint8_t pl  = raw[1];
    uint8_t ph  = raw[2];
    return ((int32_t)(int8_t)ph << 16) | ((uint16_t)pl << 8) | pxl;
}

int16_t lps_read_temperature_raw(lps_t *ps)
{
    uint8_t start = LPS_TEMP_OUT_L;
    uint8_t raw[2];

    if (ps->device == LPS_DEVICE_25H || ps->device == LPS_DEVICE_331AP) {
        start = (uint8_t)(LPS_TEMP_OUT_L | (1u << 7));
    }

    if (!i2c_write_then_read(ps, start, raw, 2)) {
        return 0;
    }

    return (int16_t)((raw[1] << 8) | raw[0]);
}

float lps_read_pressure_millibars(lps_t *ps)
{
    return (float)lps_read_pressure_raw(ps) / 4096.0f;
}

float lps_read_pressure_inches_hg(lps_t *ps)
{
    return (float)lps_read_pressure_raw(ps) / 138706.5f;
}

float lps_read_temperature_c(lps_t *ps)
{
    if (ps->device == LPS_DEVICE_22DF) {
        return (float)lps_read_temperature_raw(ps) / 100.0f;
    }
    if (ps->device == LPS_DEVICE_25H || ps->device == LPS_DEVICE_331AP) {
        return 42.5f + (float)lps_read_temperature_raw(ps) / 480.0f;
    }
    return NAN;
}

float lps_read_temperature_f(lps_t *ps)
{
    if (ps->device == LPS_DEVICE_22DF) {
        return 32.0f + (float)lps_read_temperature_raw(ps) / 100.0f * 1.8f;
    }
    if (ps->device == LPS_DEVICE_25H || ps->device == LPS_DEVICE_331AP) {
        return 108.5f + (float)lps_read_temperature_raw(ps) / 480.0f * 1.8f;
    }
    return NAN;
}

bool lps_read_data(lps_t *ps)
{
    ps->pressure = lps_read_pressure_millibars(ps);
    ps->temperature = lps_read_temperature_c(ps);
    ps->altitude = lps_pressure_to_altitude_meters(ps->pressure, 1013.25f);
    return !(isnan(ps->pressure) || isnan(ps->temperature));
}

float lps_pressure_to_altitude_meters(float pressure_mbar, float altimeter_setting_mbar)
{
    return (1.0f - powf(pressure_mbar / altimeter_setting_mbar, 0.190263f)) * 44330.8f;
}

float lps_pressure_to_altitude_feet(float pressure_inHg, float altimeter_setting_inHg)
{
    return (1.0f - powf(pressure_inHg / altimeter_setting_inHg, 0.190263f)) * 145442.0f;
}
