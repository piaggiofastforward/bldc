#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

cp force_gauge.rules /etc/udev/rules.d/
cp redboard.rules /etc/udev/rules.d/
cp vesc.rules /etc/udev/rules.d/
udevadm control --reload-rules