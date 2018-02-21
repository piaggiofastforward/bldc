#include "vesc_driver/vesc_comm.h"
#include "inttypes.h"
#include "ros/ros.h"
#include "ros/console.h"
#include <chrono>
#include <ctime>
#include "serial/serial.h"

extern "C" {
  #include "vesc_driver/packet.h"
}

namespace vesc
{

static mc_feedback_union feedback;
static mc_status_union   status;
static feedback_callback_t feedbackCallback = NULL;
static status_callback_t statusCallback = NULL;

// if we ask the VESC to run the hall calibration routine, keep track of whether or not
// we have received a response yet
static volatile bool foc_hall_detect_finished = false;

// 8 values + 1 byte (success/fail)
static volatile uint8_t foc_hall_detect_response[RESPONSE_DETECT_HALL_FOC_SIZE];

// after writing configurations, wait for a response to indicate success/failure
static volatile bool received_config_commit_confirmation = false;

// result will be 0 if unsuccessful, 1 if successful
static volatile uint8_t config_commit_result = 0;

// Settings
#define PACKET_HANDLER      0
#define MAX_BYTES_PER_READ  25

#ifdef PLATFORM_IS_LINUX
static serial::Serial ser;
#endif


/**
 * return -2 -> IOException
 * return -1 -> other error
 * return  0 -> all good
 */
int initSerial(const char* port)
{
#ifdef PLATFORM_IS_LINUX
  ser.setPort(port);
  ser.setBaudrate(vesc::UART_BAUD);
  serial::Timeout to(serial::Timeout::simpleTimeout(500));
  ser.setTimeout(to);

  try
  {
    ser.open();
    ser.close();
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
  ROS_DEBUG_STREAM("bytes written: " << bytes_written);
}

static size_t readBytes(uint8_t * dest, unsigned int max_bytes)
{
#ifdef PLATFORM_IS_LINUX
  return ser.read(dest, max_bytes);
#endif
}

/**
 * Process a received packet. Check the first byte to find out what type of packet this is,
 * then implicitly cast to the appropriate struct.
 */
static void serialProcessPacket(unsigned char *data, unsigned int length)
{
  mc_cmd cmd;
  char msg[80];

  switch (data[0])
  {
    /**
     * The first received byte is the packet identifier (FEEDBACK_DATA, STATUS_DATA, etc)
     * so copy starting at data[1] since this is the actual payload.
     */
    case FEEDBACK_DATA:
      extractFeedbackDataF(feedback, data);
      if (feedbackCallback)
        feedbackCallback(feedback.feedback);
      break;

    case CONFIG_READ:

      // this case is unimplemented until we actually have a use for it

      break;

    case STATUS_DATA:
      extractStatusDataF(status, data);
      if (statusCallback)
        statusCallback(status.status);
      break;

    /**
     * Currently, the only need for this is to verify that the VESC is correctly
     * parsing the command messages that we're giving out.
     */
    case CONTROL_WRITE:
      extractCommand(data, length, &cmd);
      switch (cmd.control_mode)
      {
        case SPEED:
        case CURRENT:
          snprintf(msg, sizeof msg, "Echo-> type: %d, value: %i, control_mode: %d\n\n",
            CONTROL_WRITE, cmd.target_cmd_i, cmd.control_mode);
          ROS_WARN(msg);
          break;
        case POSITION:
        case DUTY:
        case SCALE_POS:
        default:
          ROS_ERROR_STREAM("Received CONTROL_WRITE of unknown type " << data[1]);
          break;
      }
      break;

    /**
     *  This is the reponse to our "REQUEST_DETECT_HALL_FOC" packet.
     */
    case RESPONSE_DETECT_HALL_FOC:
      memcpy((void*)foc_hall_detect_response, data + 1, sizeof(foc_hall_detect_response));
      foc_hall_detect_finished = true;
      break;

    /**
     *  After committing configuration changes, the VESC will send us back this byte to indicate 
     *  that the configuration changes were successful.
     */
    case COMMIT_MC_CONFIG:
      received_config_commit_confirmation = true;
      config_commit_result = data[1];
      break;

    default:
      ROS_WARN("Received VESC message of unknown type\n");
      break;
  }
}

static double getTimeMillis()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(
      high_resolution_clock::now().time_since_epoch()
  ).count();
}

int commitConfiguration(unsigned int max_resp_wait_sec, bool* result)
{
  ser.flushInput();
  // command the VESC to commit the configuration
  uint8_t buf[1] = { COMMIT_MC_CONFIG };
  sendPacket(buf, 1);

  // wait a maximum of "max_resp_wait" seconds for response
  double timeBound = getTimeMillis() + (max_resp_wait_sec * 1000);
  while (!received_config_commit_confirmation)
  {
    processBytes();
    if (getTimeMillis() >= timeBound)
    {
      return -1;
    }
  }

  // commit was successful if (response == 1)
  *result = (config_commit_result == 1);
  received_config_commit_confirmation = false;
  return 0;
}

int runFocHallDetectRoutine(unsigned int max_resp_wait_sec, uint8_t* dest_buf)
{
  ser.flushInput();

  // command the VESC to start the detection routine
  uint8_t buf[1] = { REQUEST_DETECT_HALL_FOC };
  sendPacket(buf, 1);

  // wait a maximum of "max_resp_wait" seconds for it to complete
  double timeBound = getTimeMillis() + (max_resp_wait_sec * 1000);
  while (!foc_hall_detect_finished)
  {
    processBytes();
    if (getTimeMillis() >= timeBound)
    {
      return -1;
    }
  }

  memcpy(dest_buf, (void*)foc_hall_detect_response, sizeof(foc_hall_detect_response));
  foc_hall_detect_finished = false;
  return 0;
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

void disconnect()
{
  ser.flush();
  ser.close();
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