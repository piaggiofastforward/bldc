/**
 *  Run only the FOC hall detection routine, and update the config file passed in.
 *  See command_line.cpp for usage.
 */
#include "vesc_driver/vesc_comm.h"
#include "vesc_driver/vesc_ros.h"
#include "vesc_driver/vesc_config.h"
#include "vesc_driver/command_line.h"


int main(int argc, char const *argv[])
{
  if (vesc_command_line::init(argc, argv) != 0)
    return -1;
  printf("If successful, updated hall FOC table will be stored in config file...\n");
  int success = vesc_config::runHallFocDetection(argv[2], vesc::runFocHallDetectRoutine);
  vesc::disconnect();
  return success;
}