#include "vesc_driver/dyn_reconfig.h"


void DynamicReconfigure::reconfigureCallback(
  vesc_driver::vescDriverConfig &msg, 
  uint32_t lvl) 
{
  if (publish_reconfig_)
  {
    ROS_DEBUG("Reconfigure request received. Publishing...");
    dyn_pub_.publish(extractMsg(msg, lvl));
  }
}

vesc_driver::DynamicReconfigure DynamicReconfigure::extractMsg(
  vesc_driver::vescDriverConfig &srvMsg,
  uint32_t level)
{
  vesc_driver::DynamicReconfigure msg;
  msg.kp.data = srvMsg.kp;
  msg.ki.data = srvMsg.ki;
  msg.kd.data = srvMsg.kd;
  return msg;
}

// accept a dynamic reconfigure topic since rmotor and lmotor will need to share the same 
// dynamic reconfigure server
DynamicReconfigure::DynamicReconfigure(
  ros::NodeHandle *nh, std::string dyn_conf_topic,
  float kp, float ki, float kd)
      : server_("/" + dyn_conf_topic)
      , nh_(nh)
      , publish_reconfig_(false)
      , kp_(kp)
      , ki_(ki)
      , kd_(kd)
{
  ROS_WARN_STREAM("DynamicReconfigure -> using initial values: kp = " << kp_ << ", ki = " << ki_ << ", kd = " << kd_);
  dyn_pub_ = nh_->advertise<vesc_driver::DynamicReconfigure>(dyn_conf_topic, 10, true);
  server_.setCallback(
    boost::bind(&DynamicReconfigure::reconfigureCallback, this, _1, _2)
  );
}

void DynamicReconfigure::setUp() {
// first, get the default values for the motor that we're using and update the gui with them
  vesc_driver::vescDriverConfig conf;
  conf.kp = kp_;
  conf.ki = ki_;
  conf.kd = kd_;
  server_.updateConfig(conf);

  // then, publish the initial values
  publish_reconfig_ = true;
  vesc_driver::DynamicReconfigure init_msg;
  init_msg.kp.data = kp_;
  init_msg.ki.data = ki_;
  init_msg.kd.data = kd_;
  dyn_pub_.publish(init_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh_("~");
  std::string topic;
  float kp, ki, kd;
  nh_.param<std::string>("dynamic_reconfig_topic", topic, topic);
  nh_.param<float>("kp_initial", kp, kp);
  nh_.param<float>("ki_initial", ki, ki);
  nh_.param<float>("kd_initial", kd, kd);
  DynamicReconfigure srv_(&nh_, topic, kp, ki, kd);
  srv_.setUp();

  ros::Rate loopRate(30);
  while (ros::ok()) {
    ros::spinOnce();
    loopRate.sleep();
  }
}
