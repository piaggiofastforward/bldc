#include <pff_node_template.h>
#include <ros/ros.h>
#include <pff_equipment_testing/ArduinoData.h>
#include "uart_handler.h"

#include <vector>
#include <string>

class ArduinoReader : public PFFNode
{
  public:
    ros::Publisher data_publisher_;
    int device_fd_;
    std::string device_file_;
    bool connected_;
    std::vector<char> byte_stream_;
    char buffer_;

    ArduinoReader() : PFFNode()
    {
      // Get params
      ros::param::param<std::string>("~arduino_device_file", device_file_, "");

      // Setup I/O
      data_publisher_ = nh_.advertise<pff_equipment_testing::ArduinoData>("arduino_data", 10);
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
      int bytes_read = UartHandler::Read(device_fd_, &buffer_, 1);
      if (bytes_read == 1)
      {
        // Print message on CR character
        if ((int)buffer_ == CR)
        {
          pff_equipment_testing::ArduinoData data;
          timeval time_now;
          gettimeofday(&time_now, 0);
          data.timestamp.sec = time_now.tv_sec;
          data.timestamp.usec = time_now.tv_usec;
          data.reading = GetValue(byte_stream_);
          byte_stream_.clear();
          data_publisher_.publish(data);
        }

        // Ignore nulls
        else if ((int)buffer_ == NULL_CHAR || (int)buffer_ == LF)
          return;

        // Buffer message on any other character
        else
          byte_stream_.push_back(buffer_);
      }
    }

    virtual void on_shutdown()
    {
      // Do nothing
    }

    inline uint32_t GetValue(std::vector<char> byte_stream)
    {
      uint32_t integer = 0;
      for (int i = byte_stream.size() - 1; i >= 0; i--)
      {
        int value = (int)byte_stream[i] - ASCII_ZERO_OFFSET;
        double magnitude = pow(10.0, (double)((byte_stream.size() - 1) - i));
        integer += value * magnitude;
      }
      return integer;
    }
};