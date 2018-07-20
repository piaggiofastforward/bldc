#include "tcp_handler.h"
#include <thread>
#include <fstream>

class TCPServer : public TCPHandler
{
  public:
    bool running_;
    char * server_hostname_;
    int server_port_;
    std::ofstream log_file_;

    /**
     * @brief Construct a new TCPServer object
     * 
     */
    TCPServer(char * hostname, int server_port)
    {
      // Set vars
      running_ = true;
      server_hostname_ = hostname;
      server_port_ = server_port;

      // Open log file
      // log_file_.open("Aggregated_Logs.dat");
    }

    /**
     * @brief Launch point for TCPServer
     * 
     */
    void ThreadEntry()
    {
      // Assert server is active
      auto status = StartTCPServer(server_hostname_, server_port_);
      if (status != SUCCESS)
      {
        std::cout << "Error starting TCP server: " << status << std::endl;
        return;
      }

      // Poll connections
      Poll();
    }

    void Poll()
    {
      std::cout << "Beginning polling on ";
      for (int index = 0; index < strlen(server_hostname_); index++)
      {
        std::cout << server_hostname_[index];
      }
      std::cout << ":" << TCP_SERVER_PORT << std::endl;
      int fd;
      while(running_)
      {
        fd = PollTCPServer(1); // 1/1000 second timeout
        if (fd == POLL_ERROR)
        {
          std::cout << "Poll Error" << std::endl;
          return;
        }
        else if (fd == POLL_TIMEOUT)
          continue;
        else
        {
          // Get data
          int bytes_read = ReadTCP(fd);
          if (bytes_read < 0)
          {
            std::cout << "TCP read error: " << bytes_read << std::endl;
            running_ = false;
            break;
          }

          // Parse message
          auto buffer = input_buffers_[fd];
          int offset = 0;
          while (offset < buffer->size)
          {
            TCPMessageHeader * header = (TCPMessageHeader *)&buffer->data[offset];
            // std::cout << "Message length: " << (int)header->length << " offset: " << offset << std::endl;
            if (buffer->size >= header->length + offset)
            {
              // Write message to log
              log_file_.open("Aggregated_Logs.dat", std::ios::app);
              switch (header->type)
              {
                case ARDUINO_UART_READING:
                {
                  ARDUINO_UART_READING_T * arduino_data = (ARDUINO_UART_READING_T *)&buffer->data[offset];
                  std::cout << "[" << header->timestamp.tv_sec << ":" << header->timestamp.tv_usec << "] "
                    << "Got Arduino reading: " << arduino_data->reading << std::endl;

                  log_file_ << "[" << header->timestamp.tv_sec << ":" << header->timestamp.tv_usec << "] "
                  << "Arduino," << arduino_data->reading << std::endl;
                  break;
                }
                case FORCE_GAUGE_READING:
                {
                  FORCE_GAUGE_READING_T * force_data = (FORCE_GAUGE_READING_T *)&buffer->data[offset];
                  std::cout << "[" << header->timestamp.tv_sec << ":" << header->timestamp.tv_usec << "] "
                    << "Got Force reading: " << force_data->reading << std::endl;

                  log_file_ << "[" << header->timestamp.tv_sec << ":" << header->timestamp.tv_usec << "] "
                  << "Force Gauge," << force_data->reading << std::endl;
                  break;
                }
                case VESC_READING:
                {
                  VESC_READING_T * vesc_data = (VESC_READING_T *)&buffer->data[offset];
                  std::cout << "[" << header->timestamp.tv_sec << ":" << header->timestamp.tv_usec << "] "
                    << "Got Vesc reading " << std::endl
                      << "  measured_position: " << vesc_data->measured_position << std::endl
                      << "  measured_velocity: " << vesc_data->measured_velocity << std::endl
                      << "  motor_current: " << vesc_data->motor_current << std::endl
                      << "  supply_current: " << vesc_data->supply_current << std::endl
                      << "  supply_voltage: " << vesc_data->supply_voltage << std::endl;

                  log_file_ << "[" << header->timestamp.tv_sec << ":" << header->timestamp.tv_usec << "] "
                    << "Got Vesc reading " << std::endl
                      << "  measured_position: " << vesc_data->measured_position << std::endl
                      << "  measured_velocity: " << vesc_data->measured_velocity << std::endl
                      << "  motor_current: " << vesc_data->motor_current << std::endl
                      << "  supply_current: " << vesc_data->supply_current << std::endl
                      << "  supply_voltage: " << vesc_data->supply_voltage << std::endl;
                  break;
                }
                case SHUTDOWN_REQUEST:
                {
                  SHUTDOWN_REQUEST_T * shutdown_request = (SHUTDOWN_REQUEST_T *)&buffer->data[offset];
                  std::cout << "[" << header->timestamp.tv_sec << ":" << header->timestamp.tv_usec << "] "
                    << "Got shutdown request: ";
                  std::cout << "(Process): " << shutdown_request->source << std::endl;
                  running_ = false;
                  break;
                }
                default:
                {
                  std::cout << "Dropping unknown message tpye " << header->type << std::endl;
                  break;
                }
              }
              log_file_.close();
              offset += header->length;
            }
            else
            {
              // Partial mesage received, move data to the beginning and wait for more
              if (offset != 0)
              {
                buffer->size = buffer->size - offset;
                memcpy(buffer->data, &buffer->data[offset], buffer->size);
              }
              break;
            }
          }
          if (offset == buffer->size)
            buffer->size = 0;
        }
      }
    }
};