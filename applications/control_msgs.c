
#include "control_msgs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Send the command with the specified sending function
void sendCommand(packetSendFunc sendFunc, mc_cmd cmd)
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

/**
 *  In general, we can get away with implicit casting for status and feedback through the use of unions,
 *  since the size of the underlying struct is the same on x86 and cortex-m4 (through the use of specific
 *  types such as uint32_t, int16_t, etc).
 */
void sendStatusData(packetSendFunc sendFunc, const mc_status_union status)
{
  uint8_t data[sizeof(mc_status) + 1];
  data[0] = STATUS_DATA;
  memcpy(data + 1, status.status_bytes, sizeof(mc_status));
  sendFunc(data, sizeof(data));
}

void sendFeedbackData(packetSendFunc sendFunc, const mc_feedback_union fb)
{
  uint8_t data[sizeof(mc_feedback) + 1];
  data[0] = FEEDBACK_DATA;
  memcpy(data + 1, fb.feedback_bytes, sizeof(mc_feedback));
  sendFunc(data, sizeof(data));
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
      return -1;
  }
  return 0;
}

int extractStatusData(const uint8_t* data, const unsigned int size, mc_status_union *status)
{
  if (data[0] != STATUS_DATA)
  {
    return -1;
  }
  memcpy(status->status_bytes, data + 1, sizeof(mc_status));
  return 0;
}


int extractFeedbackData(const uint8_t* data, const unsigned int size, mc_feedback_union *fb)
{
  if (data[0] != FEEDBACK_DATA)
  {
    return -1;
  }
  memcpy(fb->feedback_bytes, data + 1, sizeof(mc_feedback));
  return 0;
}

int extractCurrentPIDData(const uint8_t* data, const unsigned int size, mc_config_current_pid_union *config)
{
  if (data[0] != CONFIG_WRITE_CURRENT_PID)
  {
    return -1;
  }
  memcpy(config->config_bytes, data + 1, sizeof(mc_config_current_pid));
  return 0;
}

void sendCurrentPIDData(packetSendFunc sendFunc, const mc_config_current_pid_union config)
{
  uint8_t data[sizeof(mc_config_current_pid) + 1];
  data[0] = CONFIG_WRITE_CURRENT_PID;
  memcpy(data + 1, config.config_bytes, sizeof(mc_config_current_pid));
  sendFunc(data, sizeof(data));
}
