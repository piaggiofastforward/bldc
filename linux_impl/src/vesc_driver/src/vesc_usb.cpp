#include "vesc_driver/vesc_usb.h"
#include "inttypes.h"
#include "ros/ros.h"
#include "ros/console.h"

#include "serial/serial.h"
#include "vesc_driver/control_msgs.h"

extern "C"
{
  #include "vesc_driver/platform_flags.h"
  #include "vesc_driver/packet.h"
}


namespace vesc
{

static mc_feedback_union feedback;
static mc_status_union   status;
static feedback_callback_t feedbackCallback = NULL;
static status_callback_t statusCallback = NULL;

// Settings
#define PACKET_HANDLER      0
#define MAX_BYTES_PER_READ  25

#if PLATFORM_IS_LINUX
static serial::Serial ser;
#endif


/**
 * return -2 -> IOException
 * return -1 -> other error
 * return  0 -> all good
 */
static int initSerial(const char* port)
{
#if PLATFORM_IS_LINUX
  ser.setPort(port);
  ser.setBaudrate(115200);
  serial::Timeout to(serial::Timeout::simpleTimeout(500));
  ser.setTimeout(to);

  try
  {
    ser.open();
  }
  catch (serial::IOException)
  {
    return -2;
  }

  if (!ser.isOpen()) return -1;
  return 0;
#endif
}

static void serialSendPacket(uint8_t *data, unsigned int length)
{
  size_t bytes_written = ser.write((const uint8_t*)data, length);
  ROS_WARN_STREAM("bytes written: " << bytes_written);
}

static size_t readBytes(uint8_t * dest, unsigned int max_bytes)
{
#if PLATFORM_IS_LINUX
  return ser.read(dest, max_bytes);
#endif
}

/**
 * Process a received packet. Check the first byte to find out what type of packet this is,
 * then implicitly cast to the appropriate struct.
 */
static void serialProcessPacket(unsigned char *data, unsigned int length)
{
  mc_request_union request;
  float value;
  char msg[80];

  // ROS_WARN("got some data");

  switch (data[0])
  {
    case FEEDBACK_DATA:
      /**
       * The first received byte is the packet identifier (FEEDBACK_DATA, STATUS_DATA, etc)
       * so copy starting at data[1] since this is the actual payload.
       */
      memcpy(feedback.feedback_bytes, data + 1, sizeof(mc_feedback));
      feedbackCallback(feedback.feedback);
      break;

    case CONFIG_READ:

      // not really sure what we want to do in this case...

      break;

    case STATUS_DATA:
      memcpy(status.status_bytes, data + 1, sizeof(mc_status));
      statusCallback(status.status);
      break;

    /**
     * Currently, the only need for this is to verify that the VESC is correctly
     * parsing the command messages that we're giving out.
     */
    case CONTROL_WRITE:
      memcpy(request.request_bytes, data + 1, sizeof(mc_request));
      switch(request.request.control)
      {
        case SPEED:
        case POSITION:
          value = request.request.value_i;
          break;
        case CURRENT:
        case DUTY:
        case SCALE_POS:
          value = request.request.value_f;
      }
      snprintf(msg, sizeof msg, "Echo-> type: %d, value: %f, control_mode: %d\n\n",
        request.request.type, value, request.request.control);
      ROS_WARN(msg);
      break;
    default:
      ROS_WARN("default case\n\n");

      break;
  }
}

/**
 * Use the packet handler to send packets. This will ultimately end up calling
 * the function that is passed as the *sendFunc in the call to:
 *
 *    packet_init()
 *
 * below.
 */
void sendPacket(uint8_t *data, unsigned int length)
{
  packet_send_packet(data, length, PACKET_HANDLER);
}

/**
 * Return a negative number if the serial port was not set up correctly. Otherwise,
 * return 0.
 */
int initComm(feedback_callback_t feedback_cb, status_callback_t status_cb, const char* port)
{
  int serialSuccess = initSerial(port);
  if (serialSuccess != 0)
    return serialSuccess;
  
  // pass function for sending whole packet as well as function
  // for receiving whole packet to the packet.h
  packet_init(serialSendPacket, serialProcessPacket, PACKET_HANDLER);
  feedbackCallback = feedback_cb;
  statusCallback = status_cb;
  return 0;
}

void byteReceived(uint8_t b)
{
  packet_process_byte(b, PACKET_HANDLER);
}

void onMillisTick()
{
  packet_timerfunc();
}

int processBytes()
{
  uint8_t buf[MAX_BYTES_PER_READ];
  size_t bytes_read = readBytes(buf, MAX_BYTES_PER_READ);

  if (bytes_read > 0)
  {
    for (int i = 0; i < bytes_read; ++i)
    {
      byteReceived(buf[i]);
    }
  }
  return bytes_read;
}

} // vesc