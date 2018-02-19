#include "vesc_driver/vesc_ros.h"
#include "vesc_driver/vesc_comm.h"

#define PI 3.14159265359
#define TO_RPM(x) (double(x) * 60 / (2 * M_PI))
#define FROM_RPM(x) (double(x) * (2 * M_PI) / 60)

namespace vesc
{

// M4 ROS requires this :(
#ifndef PLATFORM_IS_LINUX
static const vesc_driver::Feedback init_feedback_msg;
static const vesc_driver::Command  init_command_msg;
static const vesc_driver::Status   init_status_msg;
#endif

/**
 *  Since this is ultimately destined for an embedded platform, publishing of status and feedback
 *  will use a ring buffer (since on an embedded platform, we will receive the data in an
 *  interrupt, and it would be unwise to publish a message within that context). Whenever status
 *  or feedback data is received through vesc_usb, the corresponding process(Feedback | Status)
 *  function will be called to add the data to the ring buffer. 
 *
 *  Publish the feedback/status on the next iteration through the main loop.
 */
static vesc_driver::Feedback feedback_to_publish[FEEDBACK_BUF_SIZE];
static volatile int fbBufWriteIndex = 0;
static volatile int fbBufReadIndex = 0;

static vesc_driver::Status status_to_publish[FEEDBACK_BUF_SIZE];
static volatile int statusBufWriteIndex = 0;
static volatile int statusBufReadIndex = 0;

RosHandler::RosHandler(const char* feedback_topic, const char* status_topic,
    const char* command_topic, bool is_drive_motor, 
    ros::NodeHandle *nh
)
  : is_drive_motor_(is_drive_motor)

#ifndef PLATFORM_IS_LINUX
  , feedback_pub_(feedback_topic, &init_feedback_msg)
  , status_pub_(status_topic, &init_status_msg)
  , command_sub_(command_topic, &RosHandler::commandCallback, this)
{
  nh->advertise(feedback_pub_);
  nh->advertise(status_pub_);
  nh->subscribe(command_sub_);
};
#else
{
  feedback_pub_ = nh->advertise<vesc_driver::Feedback>(feedback_topic, 1);
  status_pub_ = nh->advertise<vesc_driver::Status>(status_topic, 1);
  command_sub_ = nh->subscribe(command_topic, 1, &RosHandler::commandCallback, this);
}
#endif

/**
 *  Receive a command callback, give it to the VESC.
 *  
 *  The commands come in as rads/s, and we want to convert to ERPM, which is
 *  equivalent to RPM * 14 for drive motors, and RPM * 8 for linear motor. 
 */
void RosHandler::commandCallback(const vesc_driver::Command::ConstPtr &msg)
{
  mc_cmd cmd;

  // multiply rpm by number of pole pairs - 14 for drive motor, 8 for linear motor
  cmd.control_mode = CURRENT;
  cmd.target_cmd_i = msg->target_cmd;
  sendCommand(sendPacket, cmd);
}

/**
 * Format and publish a ROS message based on received values from the VESC.
 */
void RosHandler::publishFeedback()
{
  if (!feedbackMessagesPending())
    return;
  feedback_pub_.publish(feedback_to_publish[fbBufReadIndex]);
  fbBufReadIndex = (fbBufReadIndex + 1) % FEEDBACK_BUF_SIZE;
}

void RosHandler::publishStatus()
{
  if (!statusMessagesPending())
    return;
  status_pub_.publish(status_to_publish[statusBufReadIndex]);
  statusBufReadIndex = (statusBufReadIndex + 1) % FEEDBACK_BUF_SIZE;
}

/**
 * Parse "values" and store the resulting Feedback message in our buffer.
 * Publish it in the main loop.
 */
void processFeedback(const mc_feedback &fb)
{
  vesc_driver::Feedback msg;
  msg.motor_current           = fb.motor_current;
  msg.measured_velocity       = fb.measured_velocity;
  msg.measured_position       = fb.measured_position;
  msg.supply_voltage          = fb.supply_voltage;
  msg.supply_current          = fb.supply_current;
  msg.switch_flags            = fb.switch_flags;

  feedback_to_publish[fbBufWriteIndex] = msg;
  fbBufWriteIndex = (fbBufWriteIndex + 1) % FEEDBACK_BUF_SIZE;
}

void processStatus(const mc_status &s)
{
  vesc_driver::Status msg;
  msg.fault_code = s.fault_code;
  msg.temp = s.temp;
  msg.limits_set = s.limits_set;

  status_to_publish[statusBufWriteIndex] = msg;
  statusBufWriteIndex = (statusBufWriteIndex + 1) % FEEDBACK_BUF_SIZE;
}

bool feedbackMessagesPending()
{
  return fbBufWriteIndex != fbBufReadIndex;
}

bool statusMessagesPending()
{
  return statusBufWriteIndex != statusBufReadIndex;
}

} // vesc