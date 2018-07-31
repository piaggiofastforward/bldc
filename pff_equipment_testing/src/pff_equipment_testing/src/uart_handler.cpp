#include "uart_handler.h"

/**
 * @brief Open a serial device
 * 
 * @param device_file 
 * @return int 
 */
int UartHandler::Open(const char *device_file)
{
  char *portname = (char *)device_file;

  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    printf("Error opening %s: %s\n", portname, strerror(errno));
    return -1;
  }

  // Baudrate 115200, 8 bits, no parity, 1 stop bit
  SetAttributes(fd, B115200);

  // Set to nonblocking mode
  fcntl(fd, F_SETFL, O_NONBLOCK);
  return fd;
}

/**
 * @brief Set UART connection attributes
 * 
 * @param fd 
 * @param speed 
 * @return int 
 */
int UartHandler::SetAttributes(int fd, int speed)
{
  struct termios tty;

  if (tcgetattr(fd, &tty) < 0)
  {
    printf("Error from tcgetattr: %s\n", strerror(errno));
    return -1;
  }

  cfsetospeed(&tty, (speed_t)speed);
  cfsetispeed(&tty, (speed_t)speed);

  tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;      /* 8-bit characters */
  tty.c_cflag &= ~PARENB;  /* no parity bit */
  tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
  tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

  /* setup for non-canonical mode */
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  /* fetch bytes as they become available */
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return -1;
  }
  return 0;
}

/**
 * @brief Send data via UART connection
 * 
 * @param data 
 * @param length 
 * @return int 
 */
int UartHandler::Write(int fd, char *data, ssize_t length)
{
  int bytes_written = write(fd, data, length);
  if (bytes_written != length)
  {
    printf("Error from write: %d, %d\n", bytes_written, errno);
    return -1;
  }
  tcdrain(fd); // Wait for write to complete
  return 0;
}

/**
 * @brief Receive data via UART connection
 * 
 * @param buffer 
 * @param length 
 * @return int 
 */
int UartHandler::Read(int fd, char * buffer, ssize_t length)
{
  int bytes_read = read(fd, buffer, length);
  return bytes_read;
}