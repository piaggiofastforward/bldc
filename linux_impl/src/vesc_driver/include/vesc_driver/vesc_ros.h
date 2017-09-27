/**
 * Separate the ROS parts of the VESC interface from the rest of the program.
 */
#include "vesc_driver/platform_flags.h"
#include "vesc_driver/Feedback.h"
#include "vesc_driver/Status.h"
#include "vesc_driver/datatypes.h"
#include "vesc_driver/Command.h"
#include "ros/ros.h"
#include "control_msgs.h"

namespace vesc
{

/**
 * Wrapper around ROS stuff, mostly just to make it easier to use this driver on
 * both the M4 and Linux.
 */
#define FEEDBACK_BUF_SIZE 16

void processFeedback(const mc_feedback &);
void processStatus(const mc_status &);

/**
 * Return true if there are unpublished feedback messages in our buffer.
 */
bool feedbackMessagesPending();
bool statusMessagesPending();

class RosHandler
{
public:
  RosHandler(const char* feedback_topic, const char* status_topic,
  	const char* command_topic, bool is_drive_motor, ros::NodeHandle *nh);

  void commandCallback(const vesc_driver::Command::ConstPtr &msg);
  void publishFeedback();
  void publishStatus();

private:
  bool is_drive_motor_;
  ros::Publisher feedback_pub_;
  ros::Publisher status_pub_;

#if PLATFORM_IS_LINUX
  ros::Subscriber command_sub_;
#else
  ros::Subscriber<vesc_driver::Command, RosHandler> command_sub_;
#endif
};

} // vesc