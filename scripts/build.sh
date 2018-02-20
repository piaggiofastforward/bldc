#! /usr/bin/env bash

# find the directory of this script, then go back one directory to find the base of the repo
# https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPTS_DIR && cd ..
REPO_DIR=$(pwd)

vesc_dir=$HOME/pff-bldc
linux_dir=$REPO_DIR/linux_impl

function stop_build {
	printf "$1"
	printf "\nFix errors and rerun.\n"
	cd $start_dir
	exit 1
}

printf "\n\n\t\tBuilding VESC firmware...\n\n"
cd $REPO_DIR && (make || stop_build "VESC build failed!")
printf "\n\n\t\tBuilding Linux driver...\n\n"
cd $linux_dir && (catkin_make || stop_build "catkin build failed!")
