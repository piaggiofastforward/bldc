#include "vesc_driver/control_msgs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Send the command with the specified sending function
void sendCommand(void (*sendFunc)(uint8_t*, unsigned int), mc_cmd cmd)
{

  /**
   *  This is to be sure we only send the bytes we need, compensating for the fact that
   *  the representation of the mc_cmd struct is different on Linux and the STM32.
   *
   *  Reserve space for the command, a byte for packet type, and a byte for control type 
   */
  const int size = sizeof(cmd.target_cmd_i) + 2;
  uint8_t data[size];
  data[0] = CONTROL_WRITE;
  data[1] = (cmd.control_mode & 0xff);
  switch (cmd.control_mode)
  {
    case SPEED:
    case CURRENT:
      memcpy(data + 2, &cmd.target_cmd_i, sizeof(cmd.target_cmd_i));
      break;
    default:
      printf("ERROR! Can only handle speed and current commands\n");
  }
  sendFunc(data, size);
}

// extract the command from a data buffer.
// Return -1 if this is not possible, and 0 otherwise
int extractCommand(const uint8_t* data, const unsigned int size, mc_cmd *cmd)
{
  if (data[0] != CONTROL_WRITE)
  {
    return -1;
  }

  cmd->control_mode = data[1];
  switch (cmd->control_mode)
  {
    case SPEED:
    case CURRENT:
      memcpy(&(cmd->target_cmd_i), data + 2, sizeof(cmd->target_cmd_i));
      break;
    default:
      printf("Only prepared to handle speed and current commands\n");
  }

}