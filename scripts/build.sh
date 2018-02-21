#! /usr/bin/env bash

BLDC_DIR=$HOME/bldc-pff
LINUX_IMPL_DIR=$BLDC_DIR/linux_impl
START_DIR=$(pwd)


function exit_build {
    printf "$1"
    printf "\n\n"
    exit 1
}

# build the vesc code first
printf "\n\n\tBuilding VESC firmware...\n\n"
cd $BLDC_DIR && (make || exit_build "pff-bldc: build failed!")

# then, build linux driver
printf "\n\n\tBuilding Linux driver...\n\n"
cd $LINUX_IMPL_DIR && (catkin_make || exit_build "vesc_driver: build failed!")
source $LINUX_IMPL_DIR/devel/setup.bash
cd $START_DIR
