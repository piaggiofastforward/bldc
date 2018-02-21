/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * app_uartcomm.c
 *
 *  Created on: 2 jul 2014
 *      Author: benjamin
 */

#include <stdio.h> // For snprintf 
#include <stdlib.h> // For atof
#include <math.h>
#include <string.h>
#include <limits.h>
#include "app.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "hw.h"
#include "packet.h"
#include "control_msgs.h"
#include "mc_interface.h"  // motor control functions
#include "timeout.h"       // timeout_reset()
#include "uart_mc_config.h"
#include "encoder.h"
#include "ext_handler.h"

// Settings
#define PACKET_HANDLER				    1
#define SERIAL_RX_BUFFER_SIZE		128
#define MAX_BYTES_PER_READ      128

// Threads
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);
static thread_t *process_tp;

/**
 * This buffer will be used for characters received through rxChar(). We should 
 * only be receiving the first 2 or 3 bytes of a packet this way - when we parse 
 *  the packet length, receive the rest of the data through uartStartReceive().
 */
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static unsigned int serial_rx_read_pos = 0;
static unsigned int serial_rx_write_pos = 0;


/** 
 * Use this buffer when calling uartStartReceive(). After that returns (via rxEnd()), take the data out
 * of this buffer and call packet_process_byte() on each byte.
 */
static uint8_t uart_receive_buffer[MAX_BYTES_PER_READ];
static int is_running = 0;


// state variables
volatile bool commandReceived      = false;
volatile bool updateConfigReceived = false;
volatile bool commitConfigReceived = false;
static volatile bool estop         = true;

/**
 * State variables for coordinating characters received through rxChar() and through uartStartReceive(),
 * to ensure that we process bytes in the actual order that we received them.
 */ 

typedef enum {
  STATE_START_BYTE = 0, // looking to receive the start byte
  STATE_PACKET_LEN,     // looking to receive the payload length
  STATE_UART_RECEIVING  // uartStartReceive() has been called
} uart_driver_state;
static volatile uart_driver_state uart_state;

// might need to change this later for REALLY BIG packets (ie: packets larger than 255)
static volatile uint8_t packetLength = 0;

/**
 * Structs to hold various transaction and state data.
 */
static volatile mc_feedback_union fb;
static volatile mc_status_union status;
static volatile mc_cmd_union cmd;
static volatile mc_cmd currentCommand;
static volatile mc_configuration mcconf;

/**
 * Forward declare various handlers.
 */
static void initHardware(void);
static void detectHallTableFoc(void);
static void setHall(hall_table_t hall_table);
static void setHallFoc(hall_table_foc_t hall_table);
static void setCommand(void);
static void sendConfigCommitConfirmation(void);
volatile float *getParamPtr(enum mc_config_param param);

/**
 * Use timer interrupts to trigger status and feedback publishing at specified intervals. Set flags in the
 * interrupts, handle in the main thread.
 */
static virtual_timer_t feedback_task_vt;
static virtual_timer_t status_task_vt;
#define isPublishing() (chVTIsArmedI(&feedback_task_vt) || chVTIsArmedI(&status_task_vt))
static void feedback_task_cb(void* _);
static void status_task_cb(void* _);
static void update_feedback(void);
static void update_status(void);

/**
 *  In response to an interrupt from the estop pin, wait to account for button debounce, then
 *  read the state through a timer interrupt. 
 */
#define ESTOP_DEBOUNCE_MS 15 // PLENTY
static virtual_timer_t estop_debounce_task_vt;
static void read_estop_cb(void* _);
static void handle_estop_interrupt(void);
/**
 *  Enable/disable the virtual timer interrupts that prompt us to send feedback/status data. This 
 *  will be used when we get an FOC detection request.
 */
static void disable_publishing(void);
static void enable_publishing(void);

/**
 *  Use this to aid in stopping/starting publishing when we receive a CONFIG_WRITE command.
 *  Schedule publishing to start up again in the amount of time below.
 */
static virtual_timer_t start_pub_task_vt;
static void start_pub_task_cb(void* _);
#define DELAY_CONFIG_WRITE_START_PUB_MS 50

/**
 * The functions that set the flags which prompt us to write status/feedback in the
 * main loop will be called at these rates. Therefore, these rates should be set slightly
 * below the desired write rate since there is a small latency between the time the flag
 * is set at the time at which the entire desired packet is sent out.
 *
 * NOTE - the feedback rate below is used as a fallback. By default, feedback will be sent to the
 * host in response to a received command.
 */ 
#define FB_RATE_MS     20  // 50Hz
#define STATUS_RATE_MS 50  // 20Hz
#define STATUS_INITIAL_DELAY ((STATUS_RATE_MS - FB_RATE_MS) / 2)
volatile bool shouldSendStatus   = false;
volatile bool shouldSendFeedback = false;

/**
 * Functions that work with the packet interface, which conveniently wraps a raw data buffer in
 * a start byte, packet length, 2 CRC, and stop bytes.
 */
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);

/**
 * For testing purposes!
 */
static void echo_command(void);
static void confirmation_echo(void);


/**
                        Function Definitions
*/

/**                     APLICATION UART DRIVER
 *
 * We can receive data in 2 ways:
 * 
 * 1) Through the rxChar() interrupt. This will be called when we receive a character
 *    but the application wasnt ready for it. Generally, this will occur on the first 
 *    two bytes of an RX transaction - we need to receive the start byte as well as 
 *    the packet length (so we know how many bytes to receive through the call to uartStartReceive()).
 *
 * 2) Through calls to uartReceive(). This will ensure a "proper" receiving of data - 
 *    the rxchar() callback will not be invoked with every incoming byte. Instead, rxEnd() 
 *    will be invoked when the number of bytes specified in the call to uartStartReceive() 
 *    is reached. In general, the number of specified bytes to receive will be equivalent 
 *    to the number of bytes left to receive in the packet. Thus, rxend() should 
 *    be invoked once the last byte of an incoming transaction is received.
 */

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp)
{
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp)
{
	(void)uartp;

	chSysLockFromISR();

	// add event flags to process_tp, probably so we can exit the chEventAwaitAny call...?
	chEvtSignalI(process_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e)
{
	(void)uartp;
	(void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 *
 * In general, we will allow the first two (or three) characters to trigger this callback. 
 *
 *    If the start byte == 2, that means the SECOND byte is the packet length.
 *    If the start byte == 3, that means the SECOND AND THIRD bytes are the packet length (len > 255)
 *
 * Store the packetLength and use this in a later call to uartStartReceive() to ensure that the rxEnd()
 * callback is invoked when we receive exactly the amount of bytes that are in an incoming message.
 */
static void rxchar(UARTDriver *uartp, uint16_t c)
{
	(void)uartp;
	serial_rx_buffer[serial_rx_write_pos++] = c;

  // this is the start byte for messages with len < 255
  if (uart_state == STATE_START_BYTE)
  {
    if ((uint8_t)c == (uint8_t)2)
    {
      uart_state = STATE_PACKET_LEN;
    }
  }

  // store the packetLength, accounting for 2 CRC bytes and a stop byte in addition
  // to the payload itself
  else if (uart_state == STATE_PACKET_LEN)
  {
    packetLength = c;
    packetLength += 3;
    uartStartReceive(&HW_UART_DEV, packetLength, uart_receive_buffer);
    uart_state = STATE_UART_RECEIVING;
  }

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chSysLockFromISR();
	chEvtSignalI(process_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/*
 * This callback is invoked when the number of bytes passed to uartStartReceive() are read.
 * Make sure to process the bytes received through rxChar() before processing these bytes...
 */
static void rxend(UARTDriver *uartp)
{
	(void)uartp;
  chSysLockFromISR();

  /*
   *  Copy received bytes from the uart_receive_buffer into the general serial_buffer
   *  so that processing of bytes received through rxchar() and rxend() happen in the 
   *  correct order
   */
  int i = 0;
  while (i < packetLength)
  {
    serial_rx_buffer[serial_rx_write_pos++] = uart_receive_buffer[i];
    if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
      serial_rx_write_pos = 0;
    }
    i++;
  }
  uart_state = STATE_START_BYTE;
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		APP_UART_BAUD,
		0,
		USART_CR2_LINEN,
		0
};

/**
 * This function will be called by packet_process_byte() when it detects the end of the packet.
 * We configure this through passing this function to packet_init().
 *
 * Extract the type of the command from the packet, and set a flag accordingly. We will then
 * perform the desired action in the main thread.
 */
static void process_packet(unsigned char *data, unsigned int len)
{
	(void)len;
  mc_config_union config;
  mc_config_hall_union config_hall;

	switch (data[0])
	{
		case CONFIG_READ:

			// currently no real need for this
			break;

		case CONTROL_WRITE:
      if (extractCommand(data, len, &currentCommand) == 0)
      {
        commandReceived = true;
        timeout_reset();

        /**
           Sync up feedback with received commands - reset the timer so that we will only
           write feedback as a result of the virtual timer if we dont receive a command
           for at least FB_RATE_MS 
        */
        if (chVTIsArmed(&feedback_task_vt))
          chVTReset(&feedback_task_vt);

        shouldSendFeedback = true;
        chVTSet(&feedback_task_vt, MS2ST(FB_RATE_MS), feedback_task_cb, NULL);
      }
			break;

		case CONFIG_WRITE:
      if (isPublishing())
      {
        disable_publishing();
      }
      // start publishing again if we dont receive a config within a specified amount of time
      chVTSet(&start_pub_task_vt, MS2ST(DELAY_CONFIG_WRITE_START_PUB_MS), start_pub_task_cb, NULL);
      memcpy(config.config_bytes, data + 1, sizeof(mc_config));
      setParameter(config.config, &mcconf);
			break;

    case CONFIG_WRITE_HALL:
      memcpy(config_hall.config_bytes, data + 1, sizeof(mc_config_hall));
      if (config_hall.config.param == HALL_TABLE)
        setHall(config_hall.config.hall_values);
      else
        setHallFoc(config_hall.config.hall_foc_values);
      break;

    case COMMIT_MC_CONFIG:
      if (isPublishing())
      {
        disable_publishing();
      }
      // at this point, dont start publishing until after we send the confirmation response
      chVTReset(&start_pub_task_vt);
      commitConfigReceived = true;
      break;

    case REQUEST_DETECT_HALL_FOC:

      // dont do things (like send feedback and accept commands to drive the motor!!!)
      disable_publishing();
      detectHallTableFoc();
      chThdSleepMilliseconds(200);
      enable_publishing();
      break;

		default:
			break;
	}
}

/**
 * Call this method to have the packet interface wrap the raw data in required headers and checksums,
 * then call its send_func() - for us, that func will be send_packet() below.
 */
static void send_packet_wrapper(unsigned char *data, unsigned int len)
{
	packet_send_packet(data, len, PACKET_HANDLER);
}

/**
 * Pass this function to packet_init() so that when we send a message, we can call
 * packet_send_packet() with the raw data inside. packet_send_packet() will wrap the raw
 * buffer data with headers, checksum, etc, then it will call this function to send
 * the properly formatted buffer.
 */
static void send_packet(unsigned char *data, unsigned int len)
{
	// Wait for the previous transmission to finish.
	while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);
	uartStartSend(&HW_UART_DEV, len, buffer);
}

/**
 * Initialize UART and estop along with EXT subsystem for hardware interrupts.
 * Our application-level callback function for handling estop state changes is
 * passed to the ext_handler, since it makes more sense for EXT to live in the
 * top-level and not within application code.
 */
static void initHardware()
{
  palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
      PAL_STM32_OSPEED_HIGHEST |
      PAL_STM32_PUDR_PULLUP);
  palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
      PAL_STM32_OSPEED_HIGHEST |
      PAL_STM32_PUDR_PULLUP);

  palClearPad(HW_ESTOP_PORT, HW_ESTOP_PIN);
  palSetPadMode(HW_ESTOP_PORT, HW_ESTOP_PIN, PAL_MODE_INPUT_PULLUP);

  estop = READ_ESTOP();
  configure_EXT();
  set_estop_callback(handle_estop_interrupt);
}

/**
 *  The packet interface provides functions to send data of our own internal message protocol
 *  and wrap it in headers, footers, and checksum.
 *
 *  Sending data:
 *  packet_send_packet() takes a payload, wraps it, and calls the first passed argument
 *  (here, send_packet()) to deal with the actual transmission of the resultant buffer.
 *
 *  Receiving data:
 *  packet_process_byte() should be called with every received byte. This runs a state machine
 *  and calls the second argument passed (here, process_packet()) to handle the received packet.   
 */
void app_uartcomm_start(void)
{

  
	packet_init(send_packet, process_packet, PACKET_HANDLER);
	uartStart(&HW_UART_DEV, &uart_cfg);
  initHardware();

  mcconf = *mc_interface_get_configuration();

  #if APP_USE_ENCODER
    encoder_init_abi(mcconf.m_encoder_counts);
  #endif

	is_running = 1;

	chThdCreateStatic(
		packet_process_thread_wa,
		sizeof(packet_process_thread_wa),
		NORMALPRIO,
		packet_process_thread,
		NULL
	);
}

void app_uartcomm_stop(void)
{
	uartStop(&HW_UART_DEV);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);

	// Notice that the processing thread is kept running in case this call is made from it.
}

void app_uartcomm_configure(uint32_t baudrate)
{
	uart_cfg.speed = baudrate;

	if (is_running) {
		uartStart(&HW_UART_DEV, &uart_cfg);
	}
}

static THD_FUNCTION(packet_process_thread, arg)
{
	(void)arg;
	chRegSetThreadName("uartcomm process");
	process_tp = chThdGetSelfX();

  /**
   * Initialize timers for feedback and status reports. These timers will set the shouldSendFeedback
   * and shouldSendStatus flags in an interrupt context, and we will do the actual
   * operation in the main thread.
   */
  chVTObjectInit(&start_pub_task_vt);
  chVTObjectInit(&status_task_vt);
  chVTObjectInit(&feedback_task_vt);
  chVTObjectInit(&estop_debounce_task_vt);
  chVTSet(&feedback_task_vt, MS2ST(FB_RATE_MS), feedback_task_cb, NULL);
  chVTSet(&status_task_vt, MS2ST(STATUS_INITIAL_DELAY), status_task_cb, NULL);

	while (1)
	{
		chEvtWaitAny((eventmask_t) 1);

    while (serial_rx_read_pos != serial_rx_write_pos)
    {

      /**
       * This should be called on every received byte. When the packet interface detects the end of
       * a packet, it will call its process_func - in our case, this will be process_packet() above. 
       */
      packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], PACKET_HANDLER);

      if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
        serial_rx_read_pos = 0;
      }
    }

    /**
     * These flags will be set in timer interrupts at rates configurable via FB_RATE_MS and STATUS_RATE_MS
     */
		if (shouldSendFeedback)
    {
      update_feedback();
      sendFeedbackData(send_packet_wrapper, fb);
      shouldSendFeedback = false;
    }
    if (shouldSendStatus)
    {
      update_status();
      sendStatusData(send_packet_wrapper, status);
      shouldSendStatus = false;
    }

    // don't give commands unless the estop is not pressed down
    if (commandReceived && !estop)
		{
			setCommand();
			commandReceived = false;
		}

    if (commitConfigReceived)
    {
      disable_publishing();
      conf_general_store_mc_configuration((mc_configuration *) &mcconf);
      mc_interface_set_configuration((mc_configuration *) &mcconf);
      chThdSleepMilliseconds(500);
      sendConfigCommitConfirmation();
      commitConfigReceived = false;
      enable_publishing();
    }
	}
}



/*****************************************************************************
	
													Implementation-specific

*****************************************************************************/

/**
 * For testing purposes, you can have the VESC echo back a received request
 * instead of actually issuing the command
 */
static void echo_command()
{
  sendCommand(send_packet_wrapper, currentCommand);
}

static void confirmation_echo()
{
  uint8_t confirmationBuf[3] = {0, 0, 0};
  send_packet_wrapper(confirmationBuf, 3);
}

/**
 *  After committing some mc_configuration changes, send data back to the host to verify that
 *  it has been completed.
 */
static void sendConfigCommitConfirmation(void)
{
  uint8_t data[2] = { COMMIT_MC_CONFIG, 1 };
  send_packet_wrapper(data, 2);
}

// stop the timer interrupts that cause us to gather and publish feedback data
static void disable_publishing(void)
{
  chVTReset(&feedback_task_vt);
  chVTReset(&status_task_vt);
}

static void enable_publishing(void)
{
  chVTSetI(&feedback_task_vt, MS2ST(FB_RATE_MS), feedback_task_cb, NULL);
  chVTSetI(&status_task_vt, MS2ST(STATUS_RATE_MS), status_task_cb, NULL);
}

/**
 * Set flags to indicate that we should publish feedback or status, handle in main thread.
 */
static void start_pub_task_cb(void* _)
{
  (void)_;
  chSysLockFromISR();
  enable_publishing();
  chSysUnlockFromISR();
}

/**
 * Set flags to indicate that we should publish feedback or status, handle in main thread.
 */
static void feedback_task_cb(void* _)
{
  (void)_;
  shouldSendFeedback = true;
  chSysLockFromISR();
  chVTSetI(&feedback_task_vt, MS2ST(FB_RATE_MS), feedback_task_cb, NULL);
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

static void status_task_cb(void* _)
{
  (void)_;
  shouldSendStatus = true;
  chSysLockFromISR();
  chVTSetI(&status_task_vt, MS2ST(STATUS_RATE_MS), status_task_cb, NULL);
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

/*
 * Updates the feedback struct with current data
 */ 
static void update_feedback(void)
{
  fb.feedback.motor_current     = mc_interface_get_tot_current();
  fb.feedback.measured_velocity = mc_interface_get_rpm();
  fb.feedback.measured_position = encoder_abs_count();
  fb.feedback.supply_voltage    = GET_INPUT_VOLTAGE();
  fb.feedback.supply_current    = mc_interface_get_tot_current_in();
  fb.feedback.estop             = estop;
}

static void update_status(void)
{
  status.status.fault_code = mc_interface_get_fault();
}

static void setCommand()
{
  switch (currentCommand.control_mode) {
    case SPEED:
      mc_interface_set_pid_speed((float)currentCommand.target_cmd_i);
      break;
    case CURRENT:
      mc_interface_set_current((float)currentCommand.target_cmd_i / 1000.0);
      break;
    default:
      break;
  }
}

/*
 * Copies the provided hall table to the local configuration
 *
 * Does not update the configuration externally
 */
static void setHall(hall_table_t hall_table)
{
  memcpy((void *) mcconf.hall_table, hall_table, HALL_TABLE_SIZE);
}

static void setHallFoc(hall_table_foc_t hall_table)
{
  memcpy((void *) mcconf.foc_hall_table, hall_table, HALL_TABLE_SIZE);
}

/**
 *  Similar to the VESC tool, perform a detection routine for FOC hall table values.
 *  The payload of the response is as follows:
 *
 *    - 1 byte  (indicating success or failure of the detection routine)
 *    - 8 bytes (hall table values)
 */
static void detectHallTableFoc(void)
{
  int index;
  mc_configuration mcconf_old, mcconf_curr;
  if (mcconf.m_sensor_port_mode == SENSOR_PORT_MODE_HALL)
  {
    mcconf_old = mcconf_curr = mcconf;
    float current = 10.0;

    mcconf_curr.motor_type = MOTOR_TYPE_FOC;
    mcconf_curr.foc_f_sw = 10000.0;
    mcconf_curr.foc_current_kp = 0.01;
    mcconf_curr.foc_current_ki = 10.0;
    mc_interface_set_configuration(&mcconf_curr);

    uint8_t hall_tab[8];
    bool res = mcpwm_foc_hall_detect(current, hall_tab);
    mc_interface_set_configuration(&mcconf_old);

    uint8_t response[RESPONSE_DETECT_HALL_FOC_SIZE + 1]; // +1 for packet ID
    index = 0;
    response[index++] = RESPONSE_DETECT_HALL_FOC;
    memcpy(response + index, hall_tab, 8);
    index += 8;
    response[index++] = res ? 1 : 0;
    send_packet_wrapper(response, 10);
  }
}

/**
 *  In case someone presses the estop quickly several times, schedule the timer so that
 *  the pin is read at time ESTOP_DEBOUNCE_MS after the last interrupt. 
 */
static void handle_estop_interrupt(void)
{
  chSysLockFromISR();
  if (chVTIsArmedI(&estop_debounce_task_vt))
  {
    chVTResetI(&estop_debounce_task_vt); 
  }
  chVTSetI(&estop_debounce_task_vt, MS2ST(ESTOP_DEBOUNCE_MS), read_estop_cb, NULL);
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

static void read_estop_cb(void* _)
{
  (void)_;
  chSysLock();
  estop = READ_ESTOP();
  chSysUnlock();
}
