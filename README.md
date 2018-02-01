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

There is code to handle this, but the messaging protocol has since changed so proper messaging will have to be re-implemented for these features.

# NOTES #

For hardware version `6.0`, there is an overlap with hardcoded ADC channels and our hardware configuration. The revolution controller has the hall sensors on pins
`PA5`, `PA6`, and `PA7` but the configuration in `hw60.c` has hardcoded 2 ADC channels on `PA5` and `PA6`.

Previous versions of the VESC hardware/firmware overwrite pins `PA5` and `PA6` from their hard coded ADC functionality and use them for `SPI` and `I2C` procotols. therefore, for the time being, we are assuming that it is okay for us to overwrite those 2 pins and use them for hall sensor readings instead. 
>>>>>>> comment out the ADC pins whose definitions overlap with our hall sensor pins
