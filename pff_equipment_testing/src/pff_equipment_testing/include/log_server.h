#include <pff_node_template.h>
#include <ros/ros.h>
#include <pff_equipment_testing/ForceGaugeData.h>
#include <pff_equipment_testing/ArduinoData.h>
#include <vesc_driver/Feedback.h>

#include <string>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

class LogServer : public PFFNode
{
  public:
    ros::Subscriber arduino_subscriber_;
    ros::Subscriber force_gauge_subscriber_;
    ros::Subscriber vesc_subscriber_;
    std::string log_location_;
    std::ofstream * log_;

    LogServer() : PFFNode()
    {
      // Get params
      ros::param::param<std::string>("~log_location", log_location_, "");
      if (log_location_[log_location_.length() - 1] != '/');
        log_location_ += "/";
      bool log_check = IsDirectory(log_location_);
      if (!log_check)
      {
        ROS_ERROR("Provided log directory does not exist, defaulting to ~/.ros/");
        log_location_ = "";
      }
      log_location_ += "Aggregated_Logs.dat";

      log_ = new std::ofstream(log_location_.c_str(), std::ios::trunc);

      // Arduino
      arduino_subscriber_ = nh_.subscribe("arduino_data", 10, &LogServer::ArduinoDataCb, this);
      add_heartbeat("arduino_data", ros::Duration(2), &LogServer::ArduinoHeartbeatCb, this);

      // Force Gauge
      force_gauge_subscriber_ = nh_.subscribe("force_gauge_data", 10, &LogServer::ForceGaugeDataCb, this);
      add_heartbeat("force_guage_data", ros::Duration(2), &LogServer::ForceGaugeHeartbeatCb, this);

      // Vesc
      vesc_subscriber_ = nh_.subscribe("/vesc_driver/feedback", 10, &LogServer::VescDataCb, this);
      add_heartbeat("vesc_data", ros::Duration(2), &LogServer::VescHeartbeatCb, this);
    }

    ~LogServer()
    {
      if (log_)
        log_->close();
        delete log_;
    }

    void ArduinoDataCb(const pff_equipment_testing::ArduinoData message)
    {
      ROS_INFO("Received Arduino data");
      *log_ << "[" << message.timestamp.sec << ":" << message.timestamp.usec << "] "
        << "Arduino Reading: "
        << message.reading << std::endl;
      heartbeat("arduino_data");
    }

    void ArduinoHeartbeatCb(const ros::TimerEvent&)
    {
      ROS_WARN("Heartbeat alert for Arduino data");
    }

    void ForceGaugeDataCb(const pff_equipment_testing::ForceGaugeData message)
    {
      ROS_INFO("Received Force Gauge data");
      *log_ << "[" << message.timestamp.sec << ":" << message.timestamp.usec << "] "
        << "Force Gauge Reading: "
        << message.reading << std::endl;
      heartbeat("force_guage_data");
    }

    void ForceGaugeHeartbeatCb(const ros::TimerEvent&)
    {
      ROS_WARN("Heartbeat alert for Force Gauge data");
    }

    void VescDataCb(const vesc_driver::Feedback message)
    {
      ROS_INFO("Received Vesc data");
      heartbeat("vesc_data");
    }

    void VescHeartbeatCb(const ros::TimerEvent&)
    {
      ROS_WARN("Heartbeat alert for Vesc data");
    }

    virtual void update()
    {
      // Do nothing
    }

    virtual void on_shutdown()
    {
      // Do nothing
    }

    inline bool IsDirectory (const std::string& name)
    {
      struct stat info;
      if( stat( name.c_str(), &info ) != 0 )
        return false;
      else if( info.st_mode & S_IFDIR )  // S_ISDIR() doesn't exist on windows 
       return true;
      else
        return false;
    }
};