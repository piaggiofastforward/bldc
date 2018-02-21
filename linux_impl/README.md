# linux_impl #

Contains the Linux driver for communication with the VESC.

## Communication ##

This driver uses the `serial::Serial` library and is intended for use along with an FTDI chip. In general, `vesc_comm.h` provides a communications interface for the rest of the driver. Currently, `vesc_comm.h` is implemented by `vesc_usb.cpp`. If this driver is to be run on another target platform which doesnt support the `serial::Serial` class, one just need to provide an alternative implementation for `vesc_comm.h` and the rest of the system can remain unchanged.

The driver uses the same packet interface as the VESC firmware (please see the `bldc` README for more info).

## Publishing Feedback and Status ##

Publishing feedback and status messages received from the VESC is achieved through function pointers passed to `vesc::initComm()`. This methodology was selected in order to decouple ROS from the communications logic in case we decide to remove the ROS component from the Linux driver.


## XML Config, Hall Table Detection over UART ##

The driver contains code to both flash an XML config file as well as detect hall table parameters over UART as an alternative to using the VESC tool connected via USB.

The following configuration executables are avilable in `pff-ros-ws/devel/lib/vesc_driver/`. All of the executables take 2 arguments: `port` and `config_file`:

~~~
config_xml      -> read an XML motor configuration file, and send it to the VESC
config_hall_foc -> perform FOC hall sensor detection routine, save values in provided XML config file
config_all      -> run config_hall_foc, then configure with config_xml
~~~

**NOTE** `config_hall_foc` does NOT send over the XML file. It just updates local configuration. `config_all` will perform the hall detection routine, save the hall table to the pointed-to config file, and upload the resulting configuration to the VESC.
