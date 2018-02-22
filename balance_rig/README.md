# balance_rig

If you haven't read up through `Things to Remember` in the top-level README, go do that now.

## Setup ##

The `motor_config` directory contains a configuration file for each motor to be used in the balance rig, and the name of each XML file corresponds directly to physical labels on each motor and **should not be used with any other motor**. 

1) Verify that each individual micro is flashed (see instructions in top-level README) with the correct code, which will most likely be from the `devel` branch.

2) Use the VESC tool to upload each micro with the configuration that corresponds to the specific motor that that micro is driving.

3) Verify that the motor drives properly through the VESC tool using the left and right arrow keys (click the arrow key icon on the right sidebar of the VESC tool to enable this command mode). If the motor doesn't drive smoothly, it is likely due to errors in RL or lambda parameters (see top-level README).


### Setup Continued - Linux Driver ###

*Note: Currently, the Linux driver serves as a direct replacement for the roboteq driver - it expects an upstream ROS node to process incoming commands, do some computation given those commands (and feedback from the driver), and publish a final command in units of **milliamps** to the `*/cmd` topic within the driver. These upstream nodes have not yet been configured for this purpose but the pipeline will likely be very similar to the flow of commands in a running Gita.*

4) Clone the repo onto the board that will be driving the rig. Build the catkin workspace within `linux_impl`.

5) Launch the triple motor launch file (check ports -  see README in `vesc_driver` package)

6) Rejoice