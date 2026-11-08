# JY901 / HWT906 Library for Raspberry Pi Pico (C)

C library for WitMotion JY901 / HWT906 sensors (Wit Standard Protocol).

## Features

- Serial packet parser (with checksum)
- Full I2C support
- **Calibration helpers** (accelerometer + magnetic field)
- **Configuration helpers** (output rate, baud rate, output content, save, factory reset)
- **Quaternion support** (read + enable)
- Conversion helpers for acc, gyro, angle, and quaternion

## Files

- `jy901.h` / `jy901.c` — Library
- `example_i2c_main.c` — I2C example
- `example_main.c` — UART example
- `CMakeLists.txt`

## I2C Quick Start

```c
jy901_t imu;
jy901_init_i2c(&imu, i2c0, 0x50);

// Read data
jy901_get_acc(&imu);
jy901_get_angle(&imu);
jy901_get_quaternion(&imu);

// Configure
jy901_set_output_rate(&imu, JY901_RATE_100HZ);
jy901_enable_quaternion(&imu);
jy901_save(&imu);
```

## Calibration

### Accelerometer
1. Place the sensor **flat and stationary** (Z-axis up).
2. Call `jy901_start_acc_calibration(&imu);`
3. Wait ~5 seconds.
4. Call `jy901_stop_calibration(&imu);`
5. Call `jy901_save(&imu);`

### Magnetic Field
1. Call `jy901_start_mag_calibration(&imu);`
2. Slowly rotate the sensor through all orientations (figure-8 or all three axes) for 10–20 seconds.
3. Call `jy901_stop_calibration(&imu);`
4. Call `jy901_save(&imu);`

## Quaternion

```c
jy901_enable_quaternion(&imu);
jy901_get_quaternion(&imu);

float q0, q1, q2, q3;
jy901_quat_to_float(&imu.quat, &q0, &q1, &q2, &q3);
// Values are in range -1.0 to +1.0
```

## Output Rate Constants

`JY901_RATE_10HZ`, `JY901_RATE_50HZ`, `JY901_RATE_100HZ`, `JY901_RATE_200HZ`, `JY901_RATE_500HZ`, `JY901_RATE_1000HZ`, etc.

## Notes

- Default I2C address: **0x50**
- Max I2C speed: **400 kHz**
- Always call `jy901_save()` after changing settings if you want them to persist
- Place sensor away from magnetic interference during mag calibration
