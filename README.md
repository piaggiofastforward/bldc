This is the source code for the VESC DC/BLDC/FOC controller. Read more at  
http://vesc-project.com/

## PFF-BLDC ##

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

# NOTES #

For hardware version `6.0`, there is an overlap with hardcoded ADC channels and our hardware configuration. The revolution controller has the hall sensors on pins
`PA5`, `PA6`, and `PA7` but the configuration in `hw60.c` has hardcoded 2 ADC channels on `PA5` and `PA6`.

Previous versions of the VESC hardware/firmware overwrite pins `PA5` and `PA6` from their hard coded ADC functionality and use them for `SPI` and `I2C` procotols. therefore, for the time being, we are assuming that it is okay for us to overwrite those 2 pins and use them for hall sensor readings instead. 
>>>>>>> comment out the ADC pins whose definitions overlap with our hall sensor pins
