# NTP-Log-Aggregator
---

TODO: Update this with full list of steps
TODO: Sync rpi thread code
TODO: Make sure shutdown is clean
TODO: Rename uart_reader to arduino reader
TODO: Change ARDUINO_UART_READING to ARDUINO_READING
TODO: Add dependencies in readme for install
TODO: redundant retry attempts
TODO: Shutdown request in vesc reader

Start by installing ntpd on raspbrry pi
``` bash
sudo apt-get install ntpd
```

Then set it up to broadcast time by editing `/etc/ntp.conf`


sudo cp redboard.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo usermod -aG dialout <username>