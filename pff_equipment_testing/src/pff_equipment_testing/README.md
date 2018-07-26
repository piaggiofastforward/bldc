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
      1. You will see messages saying: <b>Heartbeat alert has gone off for \<some node\></b>"
          - This means one of the nodes hasn't started, or shut down. Check `hw_setup.yaml` to make sure the device file is correct
      2. One of the nodes isn't configured right and won't start
          - Check that the force gauge is on and connected (This command should return a device in `/dev/`)
          ```bash
            ls /dev | grep ACM0
          ```
          - Make sure the Arduino is plugged in to the Pi
          - Unplug the Vesc from the Pi then run
          ```bash
            ls /dev | grep USB
          ```
          - This will output something like `/dev/ttyUSB0`
          - Open `bldc/pff_equipment_testing/src/pff_equipment_testing/config/hw_setup.yaml` and check that this is the value for the `arduino_device_file` parameter
          ```yaml
            # Arduino params
            arduino_device_file: /dev/ttyUSB0

            # Force gauge params
            force_gauge_device_file: /dev/ttyACM0

            # Vesc params
            port: /dev/ttyUSB1
            ```

          - Plug in the Vesc and repeat this process, but using the alternate value, for example set `port: /dev/ttyUSB1`

