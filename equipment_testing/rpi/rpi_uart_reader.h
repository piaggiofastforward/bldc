#include "tcp_handler.h"
#include "uart_handler.h"
#include <thread>

class UartReader : public TCPHandler, public UartHandler
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
      shutdown_request.source = ARDUINO_READER;
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

    UartReader(char * hostname, int server_port, char * serial_device)
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

    ~UartReader()
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
        bytes_read_ = SerialRead(buffer_, 1);
        if (bytes_read_ == 1)
        {
          // Print message on CR character
          if ((int)buffer_[0] == CR)
          {
            uint32_t data_point = GetValue(byte_stream);
            byte_stream.clear();

            ARDUINO_UART_READING_T log_data;
            log_data.header.length = sizeof(ARDUINO_UART_READING_T);
            log_data.header.type = ARDUINO_UART_READING;
            gettimeofday(&log_data.header.timestamp, 0);
            log_data.reading = data_point;
            int bytes_sent = SendTCP(fd_, &log_data, sizeof(ARDUINO_UART_READING_T));
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