/**
 *  This file is intended to be used as an interface for communication with the VESC.
 *  
 *  For desktop linux, this is done via vesc_driver/vesc_usb.cpp, and relies upon using 
 *                    Linux USB-> FTDI converter -> VESC.
 *
 *  Other platforms will need to provide their own equivalent for communication through
 *  providing definitions for the function declarations below. If this is done correctly,
 *  the rest of the source for the entire VESC driver should NOT change.
 */
#ifndef VESC_USB_H_
#define VESC_USB_H_

extern "C" {
  #include "inttypes.h"
  #include "vesc_driver/datatypes.h"
  #include "vesc_driver/control_msgs.h"
}


namespace vesc
{

typedef void(*feedback_callback_t)(const mc_feedback &);
typedef void(*status_callback_t)(const mc_status &);

constexpr const unsigned int UART_BAUD = 115200;

/**
 * Send a packet over USB
 */
void sendPacket(uint8_t *data, unsigned int length);

/**
 * Initialize USB connection, call bldc_uart_init() with function pointer
 * for sending data.
 *
 * Negative return codes represent an error condition.
 */
int initComm(feedback_callback_t feedback_cb, status_callback_t status_cb, const char* port);

/**
 *  Close the serial connection opened with initComm()
 */
void disconnect();

/**
 * Whenever a byte is received, call this function with that byte
 */
void byteReceived(uint8_t b);

/**
 * Call this function every millisecond to reset packet state machine in case
 * of potential timeouts.
 */
void onMillisTick();

/**
 * Process all newly received bytes. Return number of bytes processed.
 */
int processBytes();

/**
 *  Start detection routine, then wait for a maximum of "max_wait_sec" seconds for the detection 
 *  routine to return with values. Return values:
 *
 *    -1 -> request timed out (response not received within "sec" seconds)
 *     0 -> response received
 *
 *  It is the caller's responsibility to provide an array of size RESPONSE_DETECT_HALL_FOC_SIZE
 *  which will hold the result.
 */
int runFocHallDetectRoutine(unsigned int max_resp_wait_sec, uint8_t* dest_buf);

/**
 *  Request that the VESC commit configuration updates that we've sent over. Wait a maximum
 *  of "max_wait_resp_sec" seconds for a confirmation. Return values:
 *
 *    -1 -> request timed out (response not received within "sec" seconds)
 *     0 -> response received
 *
 *  The caller should provide a pointer to a bool to store success/fail. 
 */
int commitConfiguration(unsigned int max_resp_wait_sec, bool* result);

} // vesc

#endif // VESC_USB_H_