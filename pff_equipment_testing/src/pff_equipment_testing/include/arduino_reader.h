#include <pff_node_template.h>
#include <ros/ros.h>
#include <pff_equipment_testing/ArduinoData.h>
#include "uart_handler.h"

#include <vector>
#include <string>

// TODO: This is a total hack to get it to work well enough for Andy to use
// There is an unexplained delay every ~1ms reading the UART device which should 
// be fixed for this to be reused in the future
// Uncomment line 113 and this will immediately become noticeable
#define US_PER_SEC 350000.0
#define US_DELAY 50

inline void check_time(std::chrono::high_resolution_clock::time_point start_time, std::string tag, int arg=0)
{
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
  // if (duration > 200 || arg != 0)
  // {
  //   if (arg == 0)
  //     std::cout << "[" << tag << "]" << " took " << duration << " (us)" << std::endl;
  //   else
  //     std::cout << "[" << tag << "]" << " took " << duration << " (us)" << " count: " << arg << std::endl;
  // }
}

class ArduinoReader : public PFFNode
{
  public:
    ros::Publisher data_publisher_;
    int device_fd_;
    std::string device_file_;
    bool connected_;
    std::vector<char> byte_stream_;
    char buffer_;
    std::shared_ptr<pff_equipment_testing::ArduinoData> publish_buffer_;

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
      // Arduino updates too fast to read at a set loop rate
      // To fix this we will read quickly but only publish at the selected rate
      double publish_rate_us = US_PER_SEC * (1.0 / (double)loop_rate_);
      // ROS_WARN("Rate is %i:%i", publish_rate.sec, publish_rate.nsec);
      ros::Time last_message_time = ros::Time::now();
      ros::Time time_now;
      time_1_ = std::chrono::high_resolution_clock::now();
      std::chrono::high_resolution_clock::time_point start_time;
      publish_buffer_ = NULL;
      int counter = 0;
      while (!signal_shutdown)
      {
        check_time(start_time, "loop");


        start_time = std::chrono::high_resolution_clock::now();
        int bytes_read = UartHandler::Read(device_fd_, &buffer_, 1);
        check_time(start_time, "read");


        if (bytes_read == 1)
        {
          // Print message on CR character
          if ((int)buffer_ == CR)
          {
            start_time = std::chrono::high_resolution_clock::now();
            auto data = std::make_shared<pff_equipment_testing::ArduinoData>();
            timeval time_now;
            gettimeofday(&time_now, 0);
            data->timestamp.sec = time_now.tv_sec;
            data->timestamp.usec = time_now.tv_usec;
            data->reading = GetValue(byte_stream_);
            byte_stream_.clear();

            publish_buffer_ = data;
            check_time(start_time, "create message");
          }

          // Ignore nulls
          else if ((int)buffer_ != NULL_CHAR && (int)buffer_ != LF)
          {
            start_time = std::chrono::high_resolution_clock::now();
            byte_stream_.push_back(buffer_);
            check_time(start_time, "NULL check");
          }

          // Send things
          start_time = std::chrono::high_resolution_clock::now();
          time_2_ = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_2_ - time_1_).count();

          if (publish_buffer_ != NULL && duration >= publish_rate_us)
          {
            last_message_time = ros::Time::now();
            data_publisher_.publish(*publish_buffer_);
            check_time(time_1_, "sending", counter);
            //std::cout << "Duration: " << duration << " publish_rate_us: " << publish_rate_us << std::endl;
            publish_buffer_ = NULL;
            time_1_ = std::chrono::high_resolution_clock::now();
            counter = 0;
          }
          else
            counter++;
          check_time(start_time, "publish");
        }
        else
        {
          counter++;
          usleep(3);
        }
        start_time = std::chrono::high_resolution_clock::now();
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