/**
 *  This file is designed to be called directly from the command line, and doesnt have any ROS
 *  dependendencies. It will read the XML file passed in, and configure the VESC over UART
 *  using that configuration.
 */
#include "vesc_driver/command_line.h"
/**
 *  Upload the configuration from the passed-in XML config file to the VESC.
 *  See command_line.cpp for usage.
 */
#include "vesc_driver/vesc_config.h"
#include "vesc_driver/vesc_comm.h"

// wait 5 seconds max for the VESC to commit configuration changes and give us a response
#define MAX_CONFIG_CONFIRM_WAIT 5


int main(int argc, char const *argv[])
{
  if (vesc_command_line::init(argc, argv) != 0)
    return -1;
  int ret_val = vesc_command_line::configureWithXml(argv[2]);
  vesc::disconnect();
  return ret_val;
}