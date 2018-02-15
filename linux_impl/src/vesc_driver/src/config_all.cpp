/**
 *  Run the FOC hall detection routine, and update the config file passed in. Then, send the 
 *  updated config to the VESC, and commit the configuration.
 *  See command_line.cpp for usage.
 */
#include "vesc_driver/command_line.h"
#include "vesc_driver/vesc_comm.h"
#include <cstdio>
#include "unistd.h"


int main(int argc, char const *argv[])
{
  if (vesc_command_line::init(argc, argv) != 0)
    return -1;

  int ret_val;
  if (vesc_command_line::runFocHallDetection(argv[2]) != 0)
  {
    ret_val = -1;
  }
  else
  {
    printf("FOC hall detection successful, uploading config...\n");
    sleep(2);
    ret_val = vesc_command_line::configureWithXml(argv[2]);
  }
  vesc::disconnect();
  return ret_val;
}