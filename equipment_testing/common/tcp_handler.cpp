// Class definition for TCPHandler
#include "tcp_handler.h"

/**
 * @brief Construct a new Message Handler:: Message Handler object
 * 
 */
TCPHandler::TCPHandler()
{
  manager_fd_ = -1;
  num_connections_ = 0;

  // Create polling object
  for (int i = 0; i < MAX_CON; i++)
  {
    poll_list_[i].fd = -1;
    poll_list_[i].events = POLLIN;
    poll_list_[i].revents = 0;
  }
}

/**
 * @brief Start a TCP messging server
 * 
 * @param ip_addr 
 * @param port 
 * @return int 
 */
int TCPHandler::StartTCPServer(const char * ip_addr, int port)
{
  // Initialize socket
  int rc;
  struct sockaddr server_address;
  struct sockaddr_in *network_address = (struct sockaddr_in *)&server_address;
  int address_length = sizeof(struct sockaddr_in);

  // Set attributes
  server_address.sa_family = AF_INET;
  network_address->sin_addr.s_addr = inet_addr(ip_addr);
  network_address->sin_port = htons(port);

  // Bind to 127.0.0.1:xxxx
  manager_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (manager_fd_ < 0)
  {
    // Error creating socket
    printf("[ERROR] Unable to create requested socket - %s:%i\n", ip_addr, port);
    // return -1;
    return CREATE_SOCKET_FAILED;
  }
  int option = 1;
  rc = setsockopt(manager_fd_, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
  if (rc != 0)
  {
    // Error setting socket options
    printf("[ERROR]] Could not set socket options\n");
    // return -1;
    return SET_SOCKET_OPTIONS_FAILED;
  }

  rc = bind(manager_fd_, &server_address, address_length);
  if (rc < 0)
  {
    // Bind failed, close socket
    printf("[ERROR] Could not bind to requested socket\n");
    CloseConnection(manager_fd_);
    // return -1;
    return BIND_FAILED;
  }

  rc = listen(manager_fd_, MAX_CON);
  if (rc < 0)
  {
    // Listen failed, close socket
    printf("[ERROR] Could not listen on requested socket\n");
    CloseConnection(manager_fd_);
    // return -1;
    return LISTEN_FAILED;
  }

  // Update pollList
  poll_list_[MANAGER].fd = manager_fd_;
  poll_list_[MANAGER].events = POLLIN;
  poll_list_[MANAGER].revents = 0;
  return SUCCESS;
  // return 0
}

int TCPHandler::PollTCPServer(int timeout)
{
  int rc;
  while (true)
  {
    rc = poll(poll_list_, num_connections_ + 1, timeout); // Default infinite timeout
    if (rc == 0)
    {
      // Timeout
      return POLL_TIMEOUT;
      // return -2;
    }
    else if (rc < 0)
    {
      // Error
      return POLL_ERROR;
      // return -1;
    }

    // Read messages from established connections
    for (int i = MANAGER + 1; i < MAX_CON; i++)
    {
      if (poll_list_[i].revents != 0)
      {
        // Send fd back to function caller to read
        return poll_list_[i].fd;
      }
    }

    // Open new connections through new connection file descriptor
    if (poll_list_[MANAGER].revents != 0)
    {
      int status = ConnectionRequest();
      if (status != SUCCESS)
      {
        std::cout << "Connection request error: " << status << std::endl;
        return CONNECTION_REQUEST_FAILED;
      }
    }
  }
}

int TCPHandler::ConnectToTCPServer(const char * ipAddress, int port)
{
    // Initialize socket
    int rc;
    struct sockaddr server_address;
    struct sockaddr_in *network_address = (struct sockaddr_in *)&server_address;
    int address_length = sizeof(struct sockaddr_in);

    // Set attributes
    server_address.sa_family = AF_INET;
    network_address->sin_addr.s_addr = inet_addr(ipAddress);
    network_address->sin_port = htons(port);

    // Bind to 127.0.0.1:3737
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0)
    {
        // Error opening socket
        return OPEN_SOCKET_FAILED; // -1;
    }

    int tries = 0;
    while(tries < CONNECT_TRIES)
    {
        rc = connect(socket_fd, &server_address, address_length);
        if (rc < 0 && errno == ECONNREFUSED)
        {
            // Port connect is trying to conect to is not available yet, retry
            continue;
        }
        else if (rc < 0)
        {
            // Connect failed, close socket
            std::cout << "Connect errno: " << errno << std::endl;
            CloseConnection(socket_fd);
            return CONNECTION_CLOSED;
            // return -1;
        }
        else
        {
            // Success
            return socket_fd;
        }
    }

    return MAX_RETRIES_EXCEEDED; //-1;
}

/**
 * @brief Read TCP message
 * 
 * @param buffer 
 * @param fd 
 */
int TCPHandler::ReadTCP(int fd)
{
  // Get buffer pointer
  auto buffer = input_buffers_[fd];

  // Read into buffer with offset of current buffer size
  int bytes_read = recv(fd, (buffer->data + buffer->size), (TCP_INPUT_BUFFER_SIZE - buffer->size), MSG_DONTWAIT);

  // Error, close socket
  if (bytes_read == 0)
  {
    CloseConnection(fd);

    // Remove fd from pollList by shifting list
    bool removed = false;
    for (int index = 0; index < MAX_CON; index++)
    {
      if (removed == true)
        poll_list_[index - 1].fd = poll_list_[index].fd;
      if (poll_list_[index].fd == fd)
        removed = true;
    }
    num_connections_--;
    return CONNECTION_CLOSED;
  }
  else
  {
    buffer->size += bytes_read;
    return bytes_read;
  }
}

/**
 * @brief Handle a clients connection request
 * 
 * @return int 
 */
int TCPHandler::ConnectionRequest()
{
  int option = 1, rc;
  int connection_fd;
  struct sockaddr_in client_address;
  struct linger lngrOption = {0};
  int address_length = sizeof(struct sockaddr_in);

  connection_fd = accept(manager_fd_, (struct sockaddr *)&client_address, (unsigned int *)&address_length);
  if (connection_fd < 0)
  {
    // Accept Failed
    return ACCEPT_FAILED;
    // return -1;
  }

  if (num_connections_++ >= MAX_CON)
  {
    // Maximum number of connections reached
    num_connections_--;
    CloseConnection(connection_fd);
    return TOO_MANY_CLIENTS;
    //return 0;
  }

  // turn on keep alives
  rc = setsockopt(connection_fd, SOL_SOCKET, SO_KEEPALIVE, (char *)&option, sizeof(option));
  if (rc != 0)
  {
    // Error setting socket options
    CloseConnection(connection_fd);
    return SET_SOCKET_OPTIONS_FAILED;
    // return -1;
  }

  // turn off linger
  lngrOption.l_onoff = 0;
  rc = setsockopt(connection_fd, SOL_SOCKET, SO_LINGER, (char *)&lngrOption, sizeof(lngrOption));
  if (rc != 0)
  {
    // Error turning off linger
    CloseConnection(connection_fd);
    return SET_LINGER_FAILED;
    // return -1;
  }

  // Give new file descriptor to polling struct
  poll_list_[num_connections_].fd = connection_fd;
  poll_list_[num_connections_].revents = 0;

  // Create a buffer for the connection, then add it into the buffer map
  auto connection_buffer = std::make_shared<InputBuffer>();
  input_buffers_.insert( std::pair<int, std::shared_ptr<InputBuffer>>(connection_fd, connection_buffer));
  return SUCCESS;
}

/**
 * @brief Close a connection
 * 
 * @param fd 
 */
inline void TCPHandler::CloseConnection(int fd)
{
  // Close the file descriptor
  do
  {
  } while ((close(fd) == -1) && (errno == EINTR));

  // Remove the connecion buffer
  auto iterator = input_buffers_.find(fd);
  if (iterator != input_buffers_.end())
    input_buffers_.erase(iterator);
}