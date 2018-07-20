#!/bin/bash

server_hostname=$1
ros_setup="../../linux_impl/devel/setup.bash"

if [ ! -e $ros_setup ]; then
  echo "Project has not been build yet, exiting..."
  exit
else
  source $ros_setup

  # Check if roscore is running
  # if [ "$(pgrep roscore)" == "" ]; then
  #   echo "Roscore is not runing"
  #   roscore &
  # else
  #   echo "Roscore is already running"
  # fi

  # Start the process
  roslaunch vesc_driver driver.launch "server_hostname:=$server_hostname"
  echo "Hostname: $server_hostname"
fi