# MS5837/BAR 30 Library

Raspberry Pi library for MS5837 depth sensors using the [**Pigpiod C Interface**](http://abyz.me.uk/rpi/pigpio/pdif2.html). 

The MS5837 is a tiny SMT pressures sensor from Measurement Specialties that can measure pressure of up to 30 Bar (300m depth) with resolution of 0.2 mbar.
This library also supports the MS5837-02BA (somewhat) which has a much smaller measurement range and is better suited for altitude measurement in air.

**This library is based on the Bluerobotics Arduino and Raspberry Pi Python libraries which are linked in this repository and are also listed at the bottom of this page.

# Documentation

Check out the [**wiki**](https://git.whoi.edu/mural/drivers/bar30_depth_sensor/-/wikis/home) for more info

# Reference

- You can find the [MS5837-30BA datasheet here](http://www.mouser.com/ds/2/418/MS5837-30BA-736494.pdf).
- Original Bluerobotics Arduino Library [here](https://github.com/bluerobotics/BlueRobotics_MS5837_Library)
- Original Bluerobotics Raspberry Pi Library using SMbus [here](https://github.com/bluerobotics/ms5837-python)