
#include "control_msgs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// all messages will come in as a struct plus a single mc_packet_type byte at the front,
// with the expection of mc_cmd -> for this, we will manually extract the target command from the
// struct asw well as the control mode. The reason for this is that placing an enum inside a struct
// produces various results on different architectures (in terms of size in bytes, memory layout,
// etc) and we'd like to avoid all problems relating to those things!
#define CMD_SIZE                6   // one integer field (4 bytes) + 1 packet type + 1 control mode
#define STATUS_SIZE             sizeof(mc_status) + 1
#define FB_SIZE                 sizeof(mc_feedback) + 1
#define CURRENT_PID_CONFIG_SIZE sizeof(mc_config_current_pid) + 1

// Send the command with the specified sending function
void sendCommand(packetSendFunc sendFunc, mc_cmd cmd)
{

  /**
   *  This is to be sure we only send the bytes we need, compensating for the fact that
   *  the representation of the mc_cmd struct is different on Linux and the STM32.
   *
   *  Reserve space for the command, a byte for packet type, and a byte for control type 
   */
  uint8_t data[CMD_SIZE];
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
  sendFunc(data, CMD_SIZE);
}

/**
 *  In general, we can get away with implicit casting for status and feedback through the use of unions,
 *  since the size of the underlying struct is the same on x86 and cortex-m4 (through the use of specific
 *  types such as uint32_t, int16_t, etc).
 */
void sendStatusData(packetSendFunc sendFunc, const mc_status_union status)
{
  uint8_t data[STATUS_SIZE];
  data[0] = STATUS_DATA;
  memcpy(data + 1, status.status_bytes, sizeof(mc_status));
  sendFunc(data, sizeof(data));
}

void sendFeedbackData(packetSendFunc sendFunc, const mc_feedback_union fb)
{
  uint8_t data[FB_SIZE];
  data[0] = FEEDBACK_DATA;
  memcpy(data + 1, fb.feedback_bytes, sizeof(mc_feedback));
  sendFunc(data, sizeof(data));
}

// extract the command from a data buffer.
// Return -1 if this is not possible, and 0 otherwise
int extractCommand(const uint8_t* data, const unsigned int size, mc_cmd *cmd)
{
  if (data[0] != CONTROL_WRITE || size != CMD_SIZE)
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
  if (data[0] != STATUS_DATA || size != STATUS_SIZE)
  {
    return -1;
  }
  memcpy(status->status_bytes, data + 1, sizeof(mc_status));
  return 0;
}


int extractFeedbackData(const uint8_t* data, const unsigned int size, mc_feedback_union *fb)
{
  if (data[0] != FEEDBACK_DATA || size != FB_SIZE)
  {
    return -1;
  }
  memcpy(fb->feedback_bytes, data + 1, sizeof(mc_feedback));
  return 0;
}

int extractCurrentPIDData(const uint8_t* data, const unsigned int size, mc_config_current_pid_union *config)
{
  if (data[0] != CONFIG_WRITE_CURRENT_PID || size != CURRENT_PID_CONFIG_SIZE)
  {
    return -1;
  }
  memcpy(config->config_bytes, data + 1, sizeof(mc_config_current_pid));
  return 0;
}

void sendCurrentPIDData(packetSendFunc sendFunc, const mc_config_current_pid_union config)
{
  uint8_t data[CURRENT_PID_CONFIG_SIZE];
  data[0] = CONFIG_WRITE_CURRENT_PID;
  memcpy(data + 1, config.config_bytes, sizeof(mc_config_current_pid));
  sendFunc(data, sizeof(data));
}
