#pragma once

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <math.h>
#include <algorithm>

#define CONNECT_RETRIES 10
#define REDDBOARD_WORD_LENGTH 14
#define CRLF_WIDTH 2
#define UART_INPUT_BUFFER_SIZE 1
#define LF 10
#define CR 13
#define SUB_CHAR 45
#define NULL_CHAR 0
#define ASCII_ZERO_OFFSET 48
#define ASCII_9 57
#define ASCII_SPACE 32

class UartHandler
{
public:
  static int Open(const char *device_file);
  static int SetAttributes(int fd, int speed);
  static int Write(int fd, char *data, ssize_t length);
  static int Read(int fd, char * buffer, ssize_t length);
};