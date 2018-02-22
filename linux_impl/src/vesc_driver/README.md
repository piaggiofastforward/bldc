# vesc_driver #

Linux interface to the `VESC`.


## Configuration ##

The following configuration executables are avilable in `pff-ros-ws/devel/lib/vesc_driver/`. All of the executables take 2 arguments: `port` and `config_file`:

~~~
config_xml      -> read an XML motor configuration file, and send it to the VESC
config_hall_foc -> perform FOC hall sensor detection routine, save values in provided XML config file
config_all      -> run config_hall_foc, then configure with config_xml
~~~

**NOTE** `config_hall_foc` does NOT send over the XML file. It just updates local configuration.

The script `scripts/config_all.bash` is currently setup to configure both drive motors:

~~~
$(rospack find vesc_driver)/scripts/config_all.bash
~~~


## Running a single node ##

Ex:
~~~
roslaunch vesc_driver driver.launch motor_namespace:=rmotor port:=/dev/ttyUSB0
~~~

## Launching all 3 nodes ##

NOTE: The default ports for each motor are setup similar to Gita - ex. `/dev/pff/rmotor` - in the anticipation that someone will make udev rules for the balance rig.

Ex:
~~~
roslaunch vesc_driver driver.launch ns:=r4
~~~
