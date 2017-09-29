#include "ros/ros.h"
#include "ros/console.h"
#include "vesc_driver/vesc_usb.h"
#include "std_msgs/String.h"

#include "vesc_driver/vesc_ros.h"

void onTimerCallback(const ros::TimerEvent &)
{
  vesc::onMillisTick();
}

void refreshTimer()
{
  vesc::onMillisTick();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string port;
  std::string fb_topic;
  std::string status_topic;
  std::string cmd_topic;
  bool is_drive_motor;
  nh.param<std::string>("port", port, port);
  nh.param<std::string>("feedback_topic", fb_topic, fb_topic);
  nh.param<std::string>("status_topic", status_topic, status_topic);
  nh.param<std::string>("cmd_topic", cmd_topic, cmd_topic);
  nh.param<bool>("is_drive_motor", is_drive_motor, is_drive_motor);

  ROS_WARN_STREAM("namespace is" << nh.getNamespace() << "\n");
  ROS_WARN_STREAM("config: port [" << port << "]\n");

  vesc::RosHandler ros_handler(
    fb_topic.c_str(), status_topic.c_str(), cmd_topic.c_str(), is_drive_motor, &nh
  );

  // initialize vesc communication
  vesc::initComm(vesc::processFeedback, vesc::processStatus, port.c_str());

  ROS_WARN("Listening for VESC commands...");
  // nh.createTimer(ros::Duration(0.001), onTimerCallback);
  while(ros::ok())
  {
    refreshTimer();
    vesc::processBytes();
    if (vesc::feedbackMessagesPending())
      ros_handler.publishFeedback();
    if (vesc::statusMessagesPending())
      ros_handler.publishStatus();
    ros::spinOnce();
  }

  return 0;
}