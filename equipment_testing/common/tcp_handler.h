#pragma once
#pragma pack(0)

#include <poll.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <memory>
#include <map>
#include <sys/time.h>

#define MANAGER 0
#define MAX_CON 10
#define MAX_TCP_MSG_SIZE 2048         // bytes
#define LOCALHOST_IPV4 0x7F000001 //127.0.0.1
#define CONNECT_TRIES 5
#define TCP_INPUT_BUFFER_SIZE 2048
#define TCP_SERVER_PORT 5555

enum ErrorTypes
{
  MAX_RETRIES_EXCEEDED = -15,
  CONNECTION_CLOSED = -14,
  CONNECTION_REQUEST_FAILED = -13,
  POLL_TIMEOUT = -12,
  POLL_ERROR = -11,
  CREATE_SOCKET_FAILED = -10,
  OPEN_SOCKET_FAILED = -9,
  SET_SOCKET_OPTIONS_FAILED = -8,
  BIND_FAILED = -7,
  LISTEN_FAILED = -6,
  SET_LINGER_FAILED = -5,
  ACCEPT_FAILED = -4,
  TOO_MANY_CLIENTS = -3,
  SEND_ERROR = -2,
  RECV_ERROR= -1,
  SUCCESS = 0,
};

enum MessageTypes
{
  ARDUINO_UART_READING = 0,
  FORCE_GAUGE_READING,
  VESC_READING,
  SHUTDOWN_REQUEST
};

enum ProcessId
{
  ARDUINO_READER = 0,
  TCP_SERVER,
  FORCE_READER
};

struct TCPMessageHeader
{
  uint8_t type;
  uint8_t length;
  timeval timestamp;
};

typedef struct
{
  TCPMessageHeader header;
  uint32_t reading;
} ARDUINO_UART_READING_T;

typedef struct
{
  TCPMessageHeader header;
  int reading;
} FORCE_GAUGE_READING_T;

typedef struct
{
  TCPMessageHeader header;
  float motor_current;
  int32_t measured_velocity;
  int32_t measured_position;
  float supply_voltage;
  float supply_current;
} VESC_READING_T;

typedef struct shutdown_request
{
  TCPMessageHeader header;
  ProcessId source;

} SHUTDOWN_REQUEST_T;

class TCPHandler
{
public:
  TCPHandler();
  int StartTCPServer(const char *ip_addr, int port);
  int PollTCPServer(int timeout = -1);
  int ConnectToTCPServer(const char *ip, int port);
  template <typename TCPMessage_T>
  int SendTCP(int socket_fd, TCPMessage_T * buffer, ssize_t buffer_length)
  {
    // Send message
    int bytes_sent = send(socket_fd, buffer, buffer_length, 0); // MSG_DONTWAIT?
    if (bytes_sent <= 0 || bytes_sent != buffer_length)
      return SEND_ERROR;
    return bytes_sent;
  }
  int ReadTCP(int fd);
  inline void CloseConnection(int fd);

private:
  int ConnectionRequest();
  int manager_fd_;
  int num_connections_;
  struct pollfd poll_list_[MAX_CON];

protected:
  // Input buffering
  struct InputBuffer
  {
    int size;
    char * data;

    InputBuffer() { size = 0; data = (char *)malloc(TCP_INPUT_BUFFER_SIZE); }
    ~InputBuffer() { if(data) { free(data); }}
  };

  // Collection of buffers
  std::map<int, std::shared_ptr<InputBuffer>> input_buffers_;
};

struct TCPData
{
  int manager_fd;
  int client_fd[MAX_CON];
  struct pollfd poll_list[MAX_CON];
};