#include "ros/ros.h"
#include "ros/console.h"
#include "ros/package.h"
#include "vesc_driver/vesc_comm.h"
#include "std_msgs/String.h"
#include "vesc_driver/vesc_ros.h"
#include "vesc_driver/vesc_config.h"

#ifndef PLATFORM_IS_LINUX
  #error "Make sure you're prepared to handle this!"
#endif

int main(int argc, char** argv)
{
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string port;
  std::string fb_topic;
  std::string status_topic;
  std::string cmd_topic;
  std::string config_file;
  bool is_drive_motor;
  nh.param<std::string>("port", port, port);
  nh.param<std::string>("feedback_topic", fb_topic, fb_topic);
  nh.param<std::string>("status_topic", status_topic, status_topic);
  nh.param<std::string>("cmd_topic", cmd_topic, cmd_topic);
  nh.param<bool>("is_drive_motor", is_drive_motor, is_drive_motor);
  nh.param<std::string>("config_file", config_file, config_file);

  vesc::RosHandler ros_handler(
    fb_topic.c_str(), status_topic.c_str(), cmd_topic.c_str(), is_drive_motor, &nh
  );

  // initialize vesc communication
  if (vesc::initComm(vesc::processFeedback, vesc::processStatus, port.c_str()) != 0)
    ROS_ERROR("Error setting up serial port!");

  ROS_WARN("Listening for VESC commands...");

  nh.createTimer(
    ros::Duration(0.001),
    [] (const ros::TimerEvent &) { vesc::onMillisTick(); }
  );
  while(ros::ok())
  {
    vesc::processBytes();
    if (vesc::feedbackMessagesPending())
      ros_handler.publishFeedback();
    if (vesc::statusMessagesPending())
      ros_handler.publishStatus();
    ros::spinOnce();
  }
  vesc::disconnect();

  return 0;
}