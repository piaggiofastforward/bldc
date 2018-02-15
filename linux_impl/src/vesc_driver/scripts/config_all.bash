#! /usr/bin/env bash

config_dir=$(rospack find vesc_driver)/config
path_to_executables=~/pff-ros-ws/devel/lib/vesc_driver
config_all_executable="config_all"
drive_motor_base_fn="drive_motor.xml"
rmotor_fn="rmotor.xml"
lmotor_fn="rmotor.xml"

if [ ! -d $path_to_executables ]; then
    printf "VESC config executables not found. Please run catkin-make from pff-ros-ws\n"
    return 1
fi

# keep the drive_motor config, delete rmotor and lmotor specific files, if they exist
cd $config_dir
cp $drive_motor_base_fn $rmotor_fn
cp $drive_motor_base_fn $lmotor_fn

# run for each drive motor
printf "\nRMOTOR CONFIG----------------------------------------\n"
$path_to_executables/$config_all_executable "/dev/pff/rmotor" $config_dir/$rmotor_fn
printf "\nLmotor config----------------------------------------\n"
$path_to_executables/$config_all_executable "/dev/pff/lmotor" $config_dir/$lmotor_fn

printf "\n\nSupport for liinear motor still needed!!!!!\n"