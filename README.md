This is the source code for the VESC DC/BLDC/FOC controller. Read more at  
http://vesc-project.com/

# PFF-BLDC #

Our vesc interface is done over `UART`, and all communication functionality can be found in `applications/app_uartcomm.c`. Here are some characteristics:

- Feedback is published in response to received commands so the frequency is dictated by that of incoming data. A timer is implemented to provide a minimum feedback frequency of 50Hz if commands aren't being given.
- Status publishing at 20Hz (configurable via `STATUS_RATE_MS`)

## VESC Operation - General Instructions ##

The base of the code is provided from `vedderb/bldc`, and we have mainly built up higher-level (relatively speaking) code in the `applications` folder which leverages Benjamin's code in order to command the motor and provide feedback for the current status of the system.

The VESC tool allows a user to configure specific motors for operation. FOC is a precise algorithm - even differences between motors of the same make/model will be detected, so **each specific motor will need its own distinct configuration.** It's important to note that, whenever the VESC firmware is flashed (which may be quite frequenctly during development), IT ASSUMES THE DEFAULT CONFIGURATION, which is almost definitely not what you want! **Whenever you flash new code to the VESC you need to reflash your configuration with the VESC tool**! A well tuned motor will easily start up and drive with a `1A` setpoint, while a poorly tuned motor might not move at all.

### Things to Remember ###

Some particularly important configuration aspects to always check if the motor isn't moving smoothly:
1. hall table detection
2. RL and lambda detection (FOC only)
3. Encoder detection -> If using encoders as a commutation source instead of hall sensors, make sure to set BOTH `FOC->General->Sensor Mode = Encoder` AND
`General->Sensor Port Mode = ABI Encoder`. Then, perform the encoder detection located in `FOC->Encoder`.


## Driver and Commander ##

The folder `linux_impl` contains a Linux driver to send out commands to a VESC connected over UART. This driver uses the `serial::Serial` library and is intended for use along with an FTDI chip.

Virtually all other code in this repo pertains to the VESC firmware, including Chibi RTOS, `vedderb`'s VESC firmware on top of that, and our application firmware.

### Building/Uploading code ###

Build everything:
`./scripts/build.sh`

Linux driver:
`cd linux_impl && catkin_make`

VESC firmware:
`make` (`make upload` to flash)


# Hardware #

## Differences between TESC (Terance-ESC) and VESC HW6.0 ##

We had to make one change in `hw_60.h` to properly support the TESC. Stock VESC `6.0` does not support the simultaneous use of both encoder and hall sensors - the GPIO definitions for these pins were of the form `HW_HALL_ENC_GPIO_1`, for example. We have replaced these symbols with `HW_HALL_GPIO_1` and `HW_HALL_ENC_1` to support using both simultaneously.

Additionally, there are 2 hardcoded ADC pins in `hw_init_gpio()` within `hw_60.c` that overlapped with our hall sensor pins. Those 2 hardcoded pins have been commented out - those pins serve multiple purposes and commenting them out does not appear to effect the system in any way.


## Old hardware revisions ##

Each hardware revision has its own configuration - many pins have different purposes between different revisions. The hardware version should be silkscreened somewhere on the VESC board itself. The PFF version of the VESC uses `hw6.0`, but we do have older VESC boards around the office with versions in the `4.x` range. If these boards do not need PFF application UART code (ie: someone will give commands to the VESC via USB with the VESC tool), follow these steps to build the firmware for the specific revision:

1. clone https://github.com/vedderb/bldc
2. inside `conf_general.h`, find the section that defines the hardware revision. Make sure every hardware revision is commented out, then uncomment the line corresponding to the hardware revision of the board in question.
3. `make upload`



## UART Protocol ##

Packet protocols are like onions. There's layers. In the outermost layer we have the packet interface provided by `vedderb`, with the following structure:

```
start byte          (1)
payload length byte (1)
<payload bytes>
2 CRC bytes         (2)
end byte            (1)
```

The packet header and footer around the <`payload bytes`> are provided by the packet interface - please see `application/app_uartcomm.c` for an example of using the packet interface to both send and receive bytes.

The <`payload bytes`> follows our internal message structure. The data types of interest all reside in `applications/control_msgs.h`, and in general, all parsing of those data members should be implemented in `applications/control_msgs.c`.

```
packet_type (1)        // member of enum mc_packet_type
packet_data (variable)
```

Please see `control_msgs.h` for in-depth descriptions of each message.

## Configuration and Hall Table Detection (UART) ##

There is code to handle this, but the messaging protocol has since changed so proper messaging will have to be re-implemented for these features.
