This is the source code for the VESC DC/BLDC/FOC controller. Read more at  
http://vesc-project.com/

## PFF-BLDC ##

Our vesc interface is done over `UART`, and all communication functionality can be found in `applications/app_uartcomm.c`. Here are some characteristics:

- Feedback publishing at 50Hz (configurable via `FB_RATE_MS`)
- Status publishing at 20Hz (configurable via `STATUS_RATE_MS`)
- Command frequency is dictated by the Linux/embedded driver, but has been verified to function properly up to ~200Hz.


## VESC Operation Instructions - Current Control ##

We will be using the VESC in BLDC closed loop current control. It's important to note that, whenever the VESC is flashed, it assumes the default configuration, WHICH USES FOC MODE. Whenever you flash new code to the VESC you need to reflash your configuration with the VESC tool so that the proper operation modes are used. A sample config file is located in `config/bldc_testing.xml`.

A well tuned motor will easily start up and drive with a `1A` setpoint, while a poorly tuned motor might not move at all.

### Tuning the current control loop ###

The current control PID loop can be found in `mcpwm.c` -> `run_pid_control_current()`. Since BLDC does NOT support closed loop current control out of the box, a new data structure was added to keep track of the PID parameters. These parameters are NOT stored in non-volatile memory and will thus have to be sent over to the VESC every time a `UART` session is initiated OR whenever the VESC is powered off. This method will only be used for development and will be refined later.

`UART` communication with the VESC is done through the `vesc_driver` in `pff-ros-ws`. Current branch is `feature/vesc_driver_current_Control_pid_tuning`. After checking out that branch and building the driver, connect the VESC to the host using the on-board FTDI converter, and run the following command (note that only the port is mandatory - kp/ki/kd will all be `0.0` by default):

```
roslaunch vesc_driver pid_tuning.launch port:=$PORT kp:=$KP ki:=$KI kd:=$KD 
```

This will both start a connection with the VESC and launch a dynamic reconfigure server to be used in tuning the PID loop. View the dynamic reconfigure window with `rosrun rqt_reconfigure rqt_reconfigure`. The driver writes PID parameter values to the VESC over UART whenever they are changed (you will see this echoed in `roslaunch` logs).

After launching, commands can be given through something like

```
rostopic pub /vesc_driver/cmd vesc_driver/Command "target_cmd: 0" --rate 50
```

or by running `vesc_driver/scripts/commander.py`. This script commands the motor with 1A and 0A alternating at 10 second intervals. It's helpful for observing how an the motor responds to immediate changes in setpoint as well as observing how the actual current
reaches the setpoint over time.

Use the vesc tool to view values of interest (motor current, duty cycle). Click on the `RT` button on the right-hand side of the vesc tool window, and then click `Realtime Data` in the menu on the left. This will bring up a graphical view.


## UART Protocol ##

Packet protocols are like onions. There's layers. In the outermost layer we have the packet interface provided by `vedderb`, with the following structure:

```
start byte          (1)
payload length byte (1)
<payload bytes>
2 CRC bytes         (2)
end byte            (1)
```

The <`payload bytes`> follows our internal message structure. The data types of interest all reside in `applications/control_msgs.h`, and in general, all parsing of those data members should be implemented in `applications/control_msgs.c`.

```
packet_type (1)        // member of enum mc_packet_type
packet_data (variable)
```

Here's the packet types currently implemented (packet types are all members of `enum mc_packet_type`):

### CONTROL_WRITE ###

```
control_mode (1)        // member of enum mc_control_mode
command      (variable)

-------------------------

CONTROL_MODE       ARG LENGTH           ARG DESCRIPTION
CURRENT                4                Unsigned int representing current in MILLIAMPS
```

### FEEDBACK_DATA | STATUS_DATA ###

See the structure of these fields in the header file.

## Configuration and Hall Table Detection (UART) ##

Both of these tasks are handled through the linux driver located in `pff-ros-ws` on the branch `feature/vesc_ros_driver`.
There is code to handle this, but the messaging protocol has since changed so proper messaging will have to be re-implemented for these features.

# NOTES #

For hardware version `6.0`, there is an overlap with hardcoded ADC channels and our hardware configuration. The revolution controller has the hall sensors on pins
`PA5`, `PA6`, and `PA7` but the configuration in `hw60.c` has hardcoded 2 ADC channels on `PA5` and `PA6`.

Previous versions of the VESC hardware/firmware overwrite pins `PA5` and `PA6` from their hard coded ADC functionality and use them for `SPI` and `I2C` procotols. therefore, for the time being, we are assuming that it is okay for us to overwrite those 2 pins and use them for hall sensor readings instead. 
>>>>>>> comment out the ADC pins whose definitions overlap with our hall sensor pins
