#include "vesc_driver/vesc_usb.h"
#include "inttypes.h"
#include "ros/ros.h"
#include "ros/console.h"

#include "serial/serial.h"
#include "vesc_driver/control_msgs.h"

extern "C"
{
  #include "vesc_driver/platform_flags.h"
  // #include "vesc_driver/bldc_interface.h"
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
  // initialize USB port
  ser.setPort(port);
  ser.setBaudrate(115200);

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
  ser.write(data, (size_t)length);
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

    default:
      ROS_WARN("default case");

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


int initComm(feedback_callback_t feedback_cb, status_callback_t status_cb, const char* port)
{
  initSerial(port);
  
  // pass function for sending whole packet as well as function
  // for receiving whole packet to the packet.h
  packet_init(serialSendPacket, serialProcessPacket, PACKET_HANDLER);
  feedbackCallback = feedback_cb;
  statusCallback = status_cb;

  //bldc_interface_init(sendPacket);
  // bldc_interface_set_rx_value_func(feedback_cb);
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