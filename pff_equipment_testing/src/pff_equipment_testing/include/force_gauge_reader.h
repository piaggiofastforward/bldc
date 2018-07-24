#include <pff_node_template.h>
#include <ros/ros.h>
#include <pff_equipment_testing/ForceGaugeData.h>
#include "uart_handler.h"

#include <vector>
#include <string>

class ForceGaugeReader : public PFFNode
{
  public:
    ros::Publisher data_publisher_;
    int device_fd_;
    std::string device_file_;
    bool connected_;
    std::vector<char> byte_stream_;
    char buffer_;

    ForceGaugeReader() : PFFNode()
    {
      // Get Params
      ros::param::param<std::string>("~force_gauge_device_file", device_file_, "");
      ROS_INFO("force guage device file: %s", device_file_.c_str());

      // Setup I/O
      data_publisher_ = nh_.advertise<pff_equipment_testing::ForceGaugeData>("force_gauge_data", 10);
      connected_ = true;
      device_fd_ = UartHandler::Open(device_file_.c_str());
      if (device_fd_ < 0)
      {
        ROS_ERROR("Can't open serial device %s, quitting...", device_file_.c_str());
        connected_ = false;
      }
    }

    virtual void update()
    {
      // Force gauge updates too fast to read at a set loop rate
      // To fix this we will read quickly but only publish at the selected rate
      ros::Duration publish_rate = ros::Duration(1.0 / (double)loop_rate_);
      ros::Time last_message_time = ros::Time::now();
      ros::Time time_now;
      while (!signal_shutdown)
      {
        // Ask for data
        char get_data = '?';
        int bytes_written = UartHandler::Write(device_fd_, &get_data, sizeof(char));
        if (bytes_written < 0)
        {
          ROS_ERROR("Error writing data to %s, shutting down", device_file_.c_str());
          ros::shutdown();
        }
        else
        {
          // Read response
          int bytes_read = UartHandler::Read(device_fd_, &buffer_, 1);
          if (bytes_read == 1)
          {

            // Print message on CR character
            if ((int)buffer_ == CR)
            {
              pff_equipment_testing::ForceGaugeData data;
              timeval time_now;
              gettimeofday(&time_now, 0);
              data.timestamp.sec = time_now.tv_sec;
              data.timestamp.usec = time_now.tv_usec;
              data.reading = GetValue(byte_stream_);
              byte_stream_.clear();

              ros::Duration wait_time = ros::Duration(ros::Time::now() - last_message_time);
              if (wait_time >= publish_rate)
              {
                last_message_time = ros::Time::now();
                data_publisher_.publish(data);
              }
            }

            // Ignore nulls
            else if ((int)buffer_ == NULL_CHAR || (int)buffer_ == LF)
              return;

            // Buffer message on any other character
            else
              byte_stream_.push_back(buffer_);
          }
        }
      }
    }

    virtual void on_shutdown()
    {
      // Do nothing
    }

    inline int GetValue(std::vector<char> byte_stream)
    {
      int integer = 0;
      bool negative = false;

      // Check for negative values
      auto iter = std::find(byte_stream.begin(), byte_stream.end(), '-');
      if (iter != byte_stream.end())
      {
        negative = true;
        byte_stream.erase(iter);
      }

      // Remove everything but the value
      for (int index = 0; index < (int)byte_stream.size(); index++)
      {
        if ((int)byte_stream[index] == ASCII_SPACE)
          byte_stream.resize(index);
      }

      // Get the value
      for (int i = byte_stream.size() - 1; i >= 0; i--)
      {
        int value = (int)byte_stream[i] - ASCII_ZERO_OFFSET;
        double magnitude = pow(10.0, (double)((byte_stream.size() - 1) - i));
        integer += value * magnitude;
      }
      return (negative ? (integer * -1) : integer);
    }
};