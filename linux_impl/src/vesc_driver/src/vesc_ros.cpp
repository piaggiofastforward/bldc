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
    const char* command_topic, ros::NodeHandle *nh
)

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

  #ifdef V6_EQUIPMENT_TESTING
    // Get log server info
    nh->getParam("server_hostname", server_hostname_);
    ROS_INFO("server_hostname_: %s", server_hostname_.c_str());

    connected_ = connectToServer(server_hostname_);
  #endif
}
#endif

#ifdef V6_EQUIPMENT_TESTING
bool RosHandler::connectToServer(std::string hostname)
{
  // Connect to TCP server
  ROS_WARN("Attempting to connect to server");
  fd_ = ConnectToTCPServer(server_hostname_.c_str(), TCP_SERVER_PORT);
  if (fd_ > 0)
  {
    ROS_INFO("Connected");
    return true;
  }
  else
    return false;
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

  #ifdef V6_EQUIPMENT_TESTING
    VESC_READING_T log_data;
    log_data.header.length = sizeof(VESC_READING_T);
    log_data.header.type = VESC_READING;
    gettimeofday(&log_data.header.timestamp, 0);
    log_data.motor_current = feedback_to_publish[fbBufReadIndex].supply_current;
    log_data.measured_velocity = feedback_to_publish[fbBufReadIndex].measured_velocity;
    log_data.measured_position = feedback_to_publish[fbBufReadIndex].measured_position;
    log_data.supply_voltage = feedback_to_publish[fbBufReadIndex].supply_voltage;
    log_data.supply_current = feedback_to_publish[fbBufReadIndex].supply_current;
    int bytes_sent = SendTCP(fd_, &log_data, sizeof(VESC_READING_T));
    if (bytes_sent == SEND_ERROR)
    {
      // Server has likely stopped, so this will stop too
      connected_ = false;
      ROS_ERROR("TCP server non-responsive, shutting down");
    }
  #endif

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
  msg.estop                   = fb.estop;

  feedback_to_publish[fbBufWriteIndex] = msg;
  fbBufWriteIndex = (fbBufWriteIndex + 1) % FEEDBACK_BUF_SIZE;
}

void processStatus(const mc_status &s)
{
  vesc_driver::Status msg;
  msg.fault_code = s.fault_code;

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