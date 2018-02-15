#include "vesc_driver/command_line.h"
#include "vesc_driver/vesc_comm.h"
#include "vesc_driver/vesc_config.h"
#include "unistd.h"
#include <cstdio>

namespace vesc_command_line
{

int init(int sys_argc, const char** sys_argv)
{
  if (sys_argc != 3)
  {
    printf("Usage: provide 2 arguments: the VESC port, and the XML configuration file.\n");
    return -1;
  }

  printf("port: %s\n", sys_argv[1]);
  printf("config file: %s\n", sys_argv[2]);
  if (vesc::initComm(NULL, NULL, sys_argv[1]) != 0)
  {
    printf("\nError connecting to VESC over UART!\n");
    return -1;
  }
  return 0;
}

int configureWithXml(const char* filename)
{
  int ret_val = vesc_config::readAndSendXml(filename, vesc::sendPacket);
  if (ret_val != 0)
  {
    return ret_val;
  }

  bool commit_config_success = false;
  ret_val = 0;
  if (vesc::commitConfiguration(vesc_config::CONFIG_RESPONSE_TIMEOUT, &commit_config_success) == -1)
  {
    printf("Timed out waiting for VESC response to commit XML configuration changes!\n");
    ret_val = -1;
  }
  else if (!commit_config_success)
  {
    printf("VESC indicated failure to commit configuration changes!\n");
    ret_val = -1;
  }
  else
  {
    printf("VESC configuration successful.\n");
  }
  return ret_val;
}

int runFocHallDetection(const char* filename)
{
  printf("If successful, updated hall FOC table will be stored in config file...\n");
  return vesc_config::runHallFocDetection(filename, vesc::runFocHallDetectRoutine);
}

} //vesc_command_line