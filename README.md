This is the source code for the VESC DC/BLDC/FOC controller. Read more at  
http://vesc-project.com/

## PFF-BLDC ##

Our vesc interface is done over `UART`, and all communication functionality can be found in `applications/app_uartcomm.c`. Here are some characteristics:

- Feedback publishing at 50Hz (configurable via `FB_RATE_MS`)
- Status publishing at 20Hz (configurable via `STATUS_RATE_MS`)
- Command speed is dictated by the Linux/embedded driver, but has been verified to function properly up to ~200Hz.

Support for linear motor is in the code, but untested over `UART`.


## Motor Controller Configuration and Hall Table Calibration ##

Both of these tasks are handled through the linux driver located in `pff-ros-ws` on the branch `feature/vesc_ros_driver`

# NOTES #

For hardware version `6.0`, there is an overlap with hardcoded ADC channels and our hardware configuration. The revolution controller has the hall sensors on pins
`PA5`, `PA6`, and `PA7` but the configuration in `hw60.c` has hardcoded 2 ADC channels on `PA5` and `PA6`.

Previous versions of the VESC hardware/firmware overwrite pins `PA5` and `PA6` from their hard coded ADC functionality and use them for `SPI` and `I2C` procotols. therefore, for the time being, we are assuming that it is okay for us to overwrite those 2 pins and use them for hall sensor readings instead. 
>>>>>>> comment out the ADC pins whose definitions overlap with our hall sensor pins
