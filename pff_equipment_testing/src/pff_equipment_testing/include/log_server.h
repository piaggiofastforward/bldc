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
      log_location_ += "Aggregated_Logs.csv";

      log_ = new std::ofstream(log_location_.c_str(), std::ios::trunc);
      *log_ << "Seconds,Microseconds,Arduino Reading,Force Gauge Reading,Vesc Motor Current,Vesc Measured Velocity,Vesc Measured Position,Vesc Supply Voltage,Vesc Supply Current" << std::endl;

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

    void print_to_csv(const pff_equipment_testing::ArduinoData * arduino_data=NULL, const pff_equipment_testing::ForceGaugeData * force_gauge_data=NULL, const vesc_driver::Feedback * vesc_data=NULL)
    {
      if (arduino_data)
      {
        *log_ << arduino_data->timestamp.sec << "," << arduino_data->timestamp.usec  << "," << arduino_data->reading  << ",,,,,,," << std::endl;
      }
      if (force_gauge_data)
      {
        *log_ << force_gauge_data->timestamp.sec << "," << force_gauge_data->timestamp.usec  << ",," << force_gauge_data->reading  << ",,,,,," << std::endl;
      }
      if (vesc_data)
      {
        *log_ << vesc_data->timestamp.sec << "," << vesc_data->timestamp.usec  << ",,," << vesc_data->motor_current << "," << vesc_data->measured_velocity << "," << vesc_data->measured_position << "," << vesc_data->supply_voltage << "," << vesc_data->supply_current << std::endl;
      }

    }

    void ArduinoDataCb(const pff_equipment_testing::ArduinoData message)
    {
      ROS_INFO("Received Arduino data");
      print_to_csv(&message, NULL, NULL);
      heartbeat("arduino_data");
    }

    void ArduinoHeartbeatCb(const ros::TimerEvent&)
    {
      ROS_WARN("Heartbeat alert for Arduino data");
    }

    void ForceGaugeDataCb(const pff_equipment_testing::ForceGaugeData message)
    {
      ROS_INFO("Received Force Gauge data");
      print_to_csv(NULL, &message, NULL);
      heartbeat("force_guage_data");
    }

    void ForceGaugeHeartbeatCb(const ros::TimerEvent&)
    {
      ROS_WARN("Heartbeat alert for Force Gauge data");
    }

    void VescDataCb(const vesc_driver::Feedback message)
    {
      ROS_INFO("Received Vesc data");
      print_to_csv(NULL, NULL, &message);
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