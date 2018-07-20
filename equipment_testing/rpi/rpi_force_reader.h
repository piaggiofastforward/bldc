#include "tcp_handler.h"
#include "uart_handler.h"
#include <thread>

class ForceReader : public TCPHandler, public UartHandler
{
  public:
    bool running_;
    char * server_hostname_;
    int server_port_, fd_, bytes_read_, buffer_size_;
    char * buffer_;
    bool seek_newline_;
    char * serial_device_;

    inline void SendShutdownRequest()
    {
      SHUTDOWN_REQUEST_T shutdown_request;
      shutdown_request.source = FORCE_READER;
      shutdown_request.header.length = sizeof(SHUTDOWN_REQUEST_T);
      shutdown_request.header.type = SHUTDOWN_REQUEST;
      gettimeofday(&shutdown_request.header.timestamp, 0);
      int bytes_sent = SendTCP(fd_, &shutdown_request, sizeof(SHUTDOWN_REQUEST_T));
        if (bytes_sent == SEND_ERROR)
        {
          // Server has likely stopped, so this thread will stop too
          std::cout << "TCP send error, shutting down" << std::endl;
        }
      running_ = false;
    }

    ForceReader(char * hostname, int server_port, char * serial_device)
    {
      // Set vars
      running_ = true;
      server_hostname_ = hostname;
      server_port_ = server_port;
      buffer_size_ = 0;
      buffer_ = (char *)malloc(UART_INPUT_BUFFER_SIZE);
      memset(buffer_, 0, UART_INPUT_BUFFER_SIZE);
      seek_newline_ = true;
      serial_device_ = serial_device;
    }

    ~ForceReader()
    {
      if (buffer_)
        free(buffer_);
    }

    void ThreadEntry()
    {
      bool connect;
      connect = TCPConnect();
      if (!connect)
      {
        running_ = false;
        return;
      }

      int rc = OpenSerialDevice(serial_device_);
      if (rc != 0)
      {
        std::cout << "Can't open serial device, quitting..." << std::endl;
        SendShutdownRequest();
        running_ = false;
        return;
      }

      ReadUartData();
    }

    bool TCPConnect()
    {
      // Connect to TCP server
      for (int retries = CONNECT_RETRIES; retries > 0; retries--)
      {
        fd_ = ConnectToTCPServer(server_hostname_, server_port_);
        if (fd_ > 0)
          return true;
        else
        {
          std::cout << "Error connecting to TCP server: " << fd_ << " retries (" << retries << ")" << std::endl;
          sleep(1);
        }
      }
      return false;
    }

    void ReadUartData()
    {
      int offset;
      std::vector<char> byte_stream;
      while (running_)
      {
        char get_data = '?';
        // Ask for data
        int bytes_written = SerialWrite(&get_data, sizeof(char));
        if (bytes_written < 0)
        {
          std::cout << "Error writing data to /dev/ACM0" << std::endl;
          SendShutdownRequest();
        }
        else
        {
          // Read response
          bytes_read_ = SerialRead(buffer_, 1);
          if (bytes_read_ == 1)
          {

            // Print message on CR character
            if ((int)buffer_[0] == CR)
            {
              int data_point = GetValue(byte_stream);
              byte_stream.clear();

              FORCE_GAUGE_READING_T log_data;
              log_data.header.length = sizeof(FORCE_GAUGE_READING_T);
              log_data.header.type = FORCE_GAUGE_READING;
              gettimeofday(&log_data.header.timestamp, 0);
              log_data.reading = data_point;
              int bytes_sent = SendTCP(fd_, &log_data, sizeof(FORCE_GAUGE_READING_T));
              if (bytes_sent == SEND_ERROR)
              {
                // Server has likely stopped, so this thread will stop too
                running_ = false;
                std::cout << "TCP send error, shutting down" << std::endl;
              }
            }

            // Ignore nulls
            else if ((int)buffer_[0] == NULL_CHAR || (int)buffer_[0] == LF)
              continue;

            // Buffer message on any other character
            else
              byte_stream.push_back(buffer_[0]);
          }
        }
      }
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