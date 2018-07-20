/**
 * Separate the ROS parts of the VESC interface from the rest of the program.
 */
#include "vesc_driver/Feedback.h"
#include "vesc_driver/Status.h"
#include "vesc_driver/datatypes.h"
#include "vesc_driver/Command.h"
#include "ros/ros.h"

// Used for sending test data to log aggregator. To shut off comment out the next line
#define V6_EQUIPMENT_TESTING
#ifdef V6_EQUIPMENT_TESTING
  #include "tcp_handler.h"
#endif

extern "C" {
  #include "vesc_driver/control_msgs.h"
}

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

#ifdef V6_EQUIPMENT_TESTING
class RosHandler : public TCPHandler
#else
class RosHandler
#endif
{
public:
  RosHandler(const char* feedback_topic, const char* status_topic,
  	const char* command_topic, ros::NodeHandle *nh);

  void commandCallback(const vesc_driver::Command::ConstPtr &msg);
  void publishFeedback();
  void publishStatus();

  #ifdef V6_EQUIPMENT_TESTING
    bool connectToServer(std::string hostname);
    std::string server_hostname_;
    int fd_;
    bool connected_;
  #endif

private:
  ros::Publisher feedback_pub_;
  ros::Publisher status_pub_;

#ifdef PLATFORM_IS_LINUX
  ros::Subscriber command_sub_;
#else
  ros::Subscriber<vesc_driver::Command, RosHandler> command_sub_;
#endif
};

} // vesc