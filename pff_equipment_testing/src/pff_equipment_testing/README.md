# pff_equipment_testing
---

## Usage:
  1. Connect via ssh:
  ```bash
    # Wifi
    ssh rpi3@rpi3-3.local

    # Ethernet
    ssh rpi3@192.168.1.33
  ```
  2. First Time Setup, only required once
  ```bash
    # Add udev rules
    cd bldc/pff_equipment_testing/src/pff_equipment_testing/install
    ./install

    # Build vesc driver
    cd bldc/linux_impl
    catkin_make

    # Configure motors (With the vesc plugged in)
    cd devel/lib/vesc_driver/
    ./config_xml /dev/pff/vesc bldc/pff_equipment_testing/src/pff_equipment_testing/install/CM_Config.xml

    # Build package
    cd bldc/pff_equipment_testing
    catkin_make
  ```
  2. Source the ROS package
  ```bash
    cd bldc/pff_equipment_testing
    source devel/setup.bash
  ```
  3. (Optional) Change the log location
  ```bash
    vim bldc/pff_equipment_testing/src/pff_equipment_testing/config/hw_setup.yaml
    # Change `log_location:` from `~/.ros/` to something better
  ```
  4. Run the nodes
  ```bash
    roslaunch pff_equipment_testing test.launch
  ```

## Troubleshooting
  - Things that could go wrong
      1. You will see messages saying: <b>Heartbeat alert has gone off for \<some node\></b>
          - This means one of the nodes hasn't started, or shut down. Check `hw_setup.yaml` to make sure the device file is correct
      2. One of the nodes isn't configured right and won't start
          - Check that the device is on and connected (This command should show symlinks for all the devices)
          ```bash
            ls -sal /dev/pff
          ```
          - These will point to device files like `/dev/ttyUSB0`
          - If any conflict, run the install script `bldc/pff_equipment_testing/src/pff_equipment_testing/install/install.sh` to make sure the udev rules are properly configured
          - Also sometimes the vesc UART ports stop working, so try changing them around if the vesc does not send data

