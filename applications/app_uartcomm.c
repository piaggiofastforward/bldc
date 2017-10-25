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
#include "hw.h"
#include "packet.h"
#include "control_msgs.h"
#include "mc_interface.h"  // motor control functions
#include "timeout.h"       // timeout_reset()
#include "ext_lld.h"
#include "mcpwm_foc.h"

// Settings
#define BAUDRATE					115200
#define PACKET_HANDLER				1
#define SERIAL_RX_BUFFER_SIZE		128
#define MAX_BYTES_PER_READ      128

// Threads
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);
static thread_t *process_tp;

/**
 * This buffer will be used for characters received through rxChar(). We should only be receiving the first 2 or 3 bytes
 * of a packet this way - when we parse the packet length, receive the rest of the data through uartStartReceive().
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

/**
 * Functions that work with the packet interface, which conveniently wraps a raw data buffer in
 * a start byte, packet length, 2 CRC, and stop bytes.
 */
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);


// state variables
volatile bool rxEndReceived        = false;
volatile bool commandReceived      = false;
volatile bool updateConfigReceived = false;
static volatile bool estop = true;
static volatile bool rev_limit = false;
static volatile bool fwd_limit = false;
static volatile int32_t min_tacho = INT_MAX;
static volatile int32_t max_tacho = INT_MIN;

/**
 * Structs to hold various transaction data.
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
static void setHall(hall_table_t hall_table);
static void setHallFoc(hall_table_foc_t hall_table);
static void setCommand(void);
static void setParameter(mc_config config);
static int getStringPotValue(void);
volatile float *getParamPtr(enum mc_config_param param);
int32_t descale_position(float pos);
bool shouldMove(void);
void homing_sequence(void);
void toggle_estop(EXTDriver *extp, expchannel_t channel); 
void toggle_fwd_limit(EXTDriver *extp, expchannel_t channel);
void toggle_rev_limit(EXTDriver *extp, expchannel_t channel);


/**
 * Use timer interrupts to trigger status and feedback publishing at specified intervals. Set flags in the
 * interrupts, handle in the main thread.
 */
static virtual_timer_t feedback_task_vt;
static virtual_timer_t status_task_vt;
static void feedbackTaskCb(void* _);
static void statusTaskCb(void* _);
void sendFeedback(void);
void updateFeedback(void);
void sendStatus(void);
void updateStatus(void);

/**
 * The functions that set the flags which prompt us to write status/feedback in the
 * main loop will be called at these rates. Therefore, these rates should be set slightly
 * below the desired write rate since there is a small latency between the time the flag
 * is set at the time at which the entire desired packet is sent out.
 */ 
#define FB_RATE_MS     20  // 50Hz
#define STATUS_RATE_MS 50  // 20Hz
#define STATUS_INITIAL_DELAY ((STATUS_RATE_MS - FB_RATE_MS) / 2)
volatile bool shouldSendStatus   = false;
volatile bool shouldSendFeedback = false;


/**
 * State variables for coordinating cahracters received through rxChar() and through uartStartReceive(),
 * to ensure that we process bytes in the actual order that we received them.
 */ 

// might need to change this later for REALLY BIG packets (ie: packets larger than 255)
static volatile uint8_t packetLength = 0;
static volatile bool startByteReceived = false;
static volatile bool packetLengthReceived = false;

// set this to true after calling uartStartReceive(), set back to false in rxEnd()
static volatile bool uartReceiving = false;

/**
 * For testing purposes!
 */
void echoCommand(void);
void confirmationEcho(void);

/**
 * Each pin can have at most one interrupt across GPIO sets.
 * Each index maps to a pin.
 *
 * ESTOP     -> PC0 (pin 8)
 * FWD_LIMIT -> PA5 (pin 21)
 * REV_LIMIT -> PA6 (pin 22)
 *
 * Pass the GPIO base (eg. GPIO_A) as the first argument to underlying EXTChannelConfig
 * struct, and the index in this EXTConfig instance represents the pin index.
 */
#define ESTOP_PIN_INDEX  0
#define FWDLIM_PIN_INDEX 4
#define REVLIM_PIN_INDEX 5
#define ESTOP_PORT       GPIOC
#define FWDLIM_PORT      GPIOA
#define REVLIM_PORT      GPIOA

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, toggle_estop},
    {EXT_CH_MODE_DISABLED, NULL},    
    {EXT_CH_MODE_DISABLED, NULL},    
    {EXT_CH_MODE_DISABLED, NULL},   
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, toggle_fwd_limit},
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, toggle_rev_limit},
    {EXT_CH_MODE_DISABLED, NULL},   
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};


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
  if ((uint8_t)c == (uint8_t)2)
  {
    startByteReceived = true;
  }

  // store the packetLength, accounting for 2 CRC bytes and a stop byte
  if (startByteReceived)
  {
    packetLength = c;
    packetLength += 3;
    packetLengthReceived = true;
    startByteReceived = false;
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
	rxEndReceived = true;
  chSysLockFromISR();
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
		BAUDRATE,
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

			// need to implement
			break;

		case CONTROL_WRITE:

      memcpy(cmd.cmd_bytes, data + 1, sizeof(mc_cmd));
			currentCommand = cmd.cmd;
			commandReceived = true;
			timeout_reset();
			break;

		case CONFIG_WRITE:
      memcpy(config.config_bytes, data + 1, sizeof(mc_config));
      setParameter(config.config);
			// updateConfigReceived = true;
			break;

    case CONFIG_WRITE_HALL:
      memcpy(config_hall.config_bytes, data + 1, sizeof(mc_config_hall));
      if (config_hall.config.param == HALL_TABLE)
        setHall(config_hall.config.hall_values);
      else
        setHallFoc(config_hall.config.hall_foc_values);
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
 * Initialize estop, fwd_limit, rev_limit switches, and UART pins.
 * Initialize EXT subsystem for hardware interrupts.
 */
static void initHardware()
{
  palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
      PAL_STM32_OSPEED_HIGHEST |
      PAL_STM32_PUDR_PULLUP);
  palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
      PAL_STM32_OSPEED_HIGHEST |
      PAL_STM32_PUDR_PULLUP);

  palClearPad(ESTOP_PORT, ESTOP_PIN_INDEX);
  palClearPad(FWDLIM_PORT, FWDLIM_PIN_INDEX);
  palClearPad(REVLIM_PORT, REVLIM_PIN_INDEX);
  palSetPadMode(ESTOP_PORT, ESTOP_PIN_INDEX, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(FWDLIM_PORT, FWDLIM_PIN_INDEX, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(REVLIM_PORT, REVLIM_PIN_INDEX, PAL_MODE_INPUT_PULLUP);

  extStart(&EXTD1, &extcfg);
  estop     = palReadPad(ESTOP_PORT, ESTOP_PIN_INDEX)   == PAL_LOW;
  rev_limit = palReadPad(REVLIM_PORT, REVLIM_PIN_INDEX) == PAL_LOW;
  fwd_limit = palReadPad(FWDLIM_PORT, FWDLIM_PIN_INDEX) == PAL_LOW;
}

void app_uartcomm_start(void)
{
	packet_init(send_packet, process_packet, PACKET_HANDLER);
	uartStart(&HW_UART_DEV, &uart_cfg);
  initHardware();

  mcconf = *mc_interface_get_configuration();
  /* (void) getStringPotValue; */
  mc_interface_set_pid_pos_src(getStringPotValue);

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
   * and shouldSendStatus flags in an interrupt context, and we will do the actual data transmission
   * in the main thread.
   */
  chVTObjectInit(&status_task_vt);
  chVTObjectInit(&feedback_task_vt);
  chVTSet(&feedback_task_vt, MS2ST(FB_RATE_MS), feedbackTaskCb, NULL);
  chVTSet(&status_task_vt, MS2ST(STATUS_INITIAL_DELAY), statusTaskCb, NULL);

	while (1)
	{
		chEvtWaitAny((eventmask_t) 1);

    /**
     * We can receive data in 2 ways:
     * 
     * 1) Through the rxChar() interrupt above. This will be called when we receive a character
     *    but the application wasnt ready for it. In this case, we want to prepare the UARTDriver
     *    object to correctly receive the rest of the packet (as opposed to interrupting and calling rxChar() on
     *    every single received character), placing all the data in the uart_receive_buffer.
     *
     * 2) Through calls to uartReceive(). This will ensure a "proper" receiving of data - 
     *    the rxchar() callback will not be invoked. Instead, rxEnd() will be invoked when the number of bytes
     *    specified in the call to uartStartReceive() is reached.
     *
     *  "uartReceiving" will be set to true just after uartStartReceive() is called, and will be set back to false after
     *  all of the bytes received in the uart_receive_buffer are processed via packet_process_byte(). This guards against
     *  the condition where, at high command speeds, we process a byte received through rxChar() before processing all
     *  of the bytes received through uartStartReceive(). 
     */

    while (!uartReceiving && (serial_rx_read_pos != serial_rx_write_pos))
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
     * Handle the case where the uartStartReceive function returned (ie: the MAX_BYTES_PER_READ)
     * was met) make sure we first process the bytes received from rxChar() (while statement above)
     * and then process all of these bytes so that we call packet_process_byte() in the correct sequence.
     */

    if (rxEndReceived)
    {
      for (int i = 0; i < packetLength; i++)
      {
        packet_process_byte(uart_receive_buffer[i], PACKET_HANDLER);
      }
      rxEndReceived = false;
      uartReceiving = false;
    }

    /**
     * If we received the packetLength, use uartStartReceive() to receive the rest of the bytes. When
     * the specified number of bytes is read, rxEnd() will be invoked, setting rxEndReceived = true
     */
    if (packetLengthReceived)
    {
      uartStartReceive(&HW_UART_DEV, packetLength, uart_receive_buffer);
      uartReceiving = true;
      packetLengthReceived = false;
    }

    /**
     * These flags will be set in timer interrupts at rates configurable via FB_RATE_MS and STATUS_RATE_MS
     */
		if (shouldSendFeedback)
    {
      sendFeedback();
      shouldSendFeedback = false;
    }
    if (shouldSendStatus)
    {
      sendStatus();
      shouldSendStatus = false;
    }

    /**
     * TODO: proper handling of estop (brake? or just set command to 0?)
     */
		if (estop)
		{
      mcpwm_foc_stop_pwm();
			// mc_interface_set_brake_current(0);
		}
		else if (!shouldMove())
		{
			// mc_interface_brake_now();
		}

    else if (commandReceived)
		{
			setCommand();
			commandReceived = false;
		}

		// if (updateConfigReceived)
		// {
		// 	// do stuff
		// 	mc_interface_set_configuration((mc_configuration *) &mcconf);
		// 	conf_general_store_mc_configuration((mc_configuration *) &mcconf);
		// 	updateConfigReceived = false;
		// }
	}
}



/*****************************************************************************
	
													Implementation-specific

*****************************************************************************/

/**
 * For testing purposes, you can have the VESC echo back a received request
 * instead of actually issuing the command
 */
void echoCommand()
{
  uint8_t data[sizeof(mc_cmd) + 1];
  data[0] = CONTROL_WRITE;
  memcpy(data + 1, cmd.cmd_bytes, sizeof(mc_cmd));
  send_packet_wrapper(data, sizeof(data)); 
}

void confirmationEcho()
{
  uint8_t confirmationBuf[3] = {0, 0, 0};
  send_packet_wrapper(confirmationBuf, 3);
}

/**
 * Set flags to indicate that we should publish feedback or status, handle in main thread.
 */
static void feedbackTaskCb(void* _)
{
  (void)_;
  shouldSendFeedback = true;
  chSysLockFromISR();
  chVTSetI(&feedback_task_vt, MS2ST(FB_RATE_MS), feedbackTaskCb, NULL);
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

static void statusTaskCb(void* _)
{
  (void)_;
  shouldSendStatus = true;
  chSysLockFromISR();
  chVTSetI(&status_task_vt, MS2ST(STATUS_RATE_MS), statusTaskCb, NULL);
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

/*
 * Updates the feedback struct with current data
 */ 
void updateFeedback(void)
{
  fb.feedback.motor_current     = mc_interface_get_tot_current();
  fb.feedback.measured_velocity = mc_interface_get_rpm();
  fb.feedback.measured_position = mc_interface_get_pid_pos_now();
  fb.feedback.supply_voltage    = GET_INPUT_VOLTAGE();
  fb.feedback.supply_current    = mc_interface_get_tot_current_in();
  fb.feedback.switch_flags      = (estop << 2) | (rev_limit << 1) | (fwd_limit); 
}

/**
 * Send our current feedback data.
 */
void sendFeedback(void)
{
	updateFeedback();
  uint8_t data[sizeof(mc_feedback) + 1];
  data[0] = FEEDBACK_DATA;
  memcpy(data + 1, fb.feedback_bytes, sizeof(mc_feedback));
	send_packet_wrapper(data, sizeof(data));
}

void updateStatus(void)
{
  status.status.fault_code = mc_interface_get_fault();
  status.status.temp       = NTC_TEMP(ADC_IND_TEMP_MOS);
  status.status.limits_set = min_tacho != INT_MAX && max_tacho != INT_MIN;
}

void sendStatus(void)
{
  updateStatus();
  uint8_t data[sizeof(mc_status) + 1];
  data[0] = STATUS_DATA;
  memcpy(data + 1, status.status_bytes, sizeof(mc_status));
  send_packet_wrapper(data, sizeof(data));
}

void setCommand()
{
  switch (currentCommand.control_mode) {
    case SPEED:
      fb.feedback.commanded_value = currentCommand.target_cmd_i;
      // echoCommand();
      mc_interface_set_pid_speed(currentCommand.target_cmd_f);
      break;
    // case CURRENT:
    //   fb.feedback.commanded_value = currentCommand.target_cmd_f * 1000; 
    //   mc_interface_set_current(currentCommand.target_cmd_f);
    //   break;
    // case DUTY:
    //   fb.feedback.commanded_value = currentCommand.target_cmd_f * 1000;
    //   mc_interface_set_duty(currentCommand.target_cmd_f);
    //   break;
    // case POSITION:
    //   fb.feedback.commanded_value = currentCommand.target_cmd_i;
    //   mc_interface_set_pid_pos(currentCommand.target_cmd_i);
    //   break;
    // case SCALE_POS:
    //   fb.feedback.commanded_value = currentCommand.target_cmd_f * 1000;
    //   mc_interface_set_pid_pos(descale_position(currentCommand.target_cmd_f));
    //   break;
    // case HOMING: 
    //   fb.feedback.commanded_value = mc_interface_get_pid_pos_now();
    //   homing_sequence();
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


/*
 * Returns the absolute position in ticks based on the min and max limits
 * pos should be between -1 and 1
 */
int32_t descale_position(float pos)
{
    int32_t offset = (max_tacho - min_tacho) * pos / 2;
    int32_t center = (max_tacho + min_tacho) / 2;
    return center + offset;
}

/*
 * A state machine to determine the limits of position control using limit switches
 *
 * TODO: Make current control value configurable
 */
void homing_sequence(void) 
{
  if (max_tacho == INT_MIN) {
    if (!fwd_limit) {
      mc_interface_set_current(8);
    } 
  } else if (min_tacho == INT_MAX) {
    if (!rev_limit) {
      mc_interface_set_current(-8);
    }
  }
}

/*
 * Returns true if the limit switch is not set in the desired direction
 */
bool shouldMove(void) 
{
  bool isForward = true;
  switch (currentCommand.control_mode) {
    case POSITION:
      isForward = currentCommand.target_cmd_i - mc_interface_get_tachometer_value(false) > 0;
      break;
    case SCALE_POS:
      isForward = descale_position(currentCommand.target_cmd_f / 1000.0f) > 0 - 
        mc_interface_get_tachometer_value(false);
      break;
    case SPEED:
      isForward = currentCommand.target_cmd_i > 0;
      break;
    case HOMING:
      if (max_tacho == INT_MIN) {
        isForward = true;
      } else if (min_tacho == INT_MAX) {
        isForward = false;
      }
    case CURRENT:
    case DUTY:
      isForward = currentCommand.target_cmd_f > 0;
      break;
  }

  // Return false if the limit switch for the current direction is triggered
  return !(isForward ? fwd_limit : rev_limit);
}

/*
 * Reads the estop value and disables control if it is set
 * Called from interrupt context on both edges
 * Pulldown input, so unless externally pulled up, default value is PAL_LOW
 */
void toggle_estop(EXTDriver *extp, expchannel_t channel) 
{
  (void) extp;
  (void) channel;
  estop = palReadPad(ESTOP_PORT, ESTOP_PIN_INDEX) == PAL_LOW;
  // if (estop) {
  //   mc_interface_set_brake_current(0);
  // }
}

/*
 * Reads the forward limit switch. If set and trying to move forward, brake
 * Sets the max_tacho limit
 * Called from interrupt context on both edges
 */
void toggle_fwd_limit(EXTDriver *extp, expchannel_t channel) 
{
  (void) extp;
  (void) channel;
  fwd_limit = palReadPad(FWDLIM_PORT, FWDLIM_PIN_INDEX) == PAL_LOW;
  if (fwd_limit) {
    if (!shouldMove()) {
      // mc_interface_brake_now();
    }
    max_tacho = mc_interface_get_tachometer_value(false);
  }
}

/*
 * Reads the reverse limit switch. If set and trying to move backwards, brake
 * Sets the min_tacho limit
 * Called from interrupt context on both edges
 */
void toggle_rev_limit(EXTDriver *extp, expchannel_t channel) 
{
  (void) extp;
  (void) channel;
  rev_limit = palReadPad(REVLIM_PORT, REVLIM_PIN_INDEX) == PAL_LOW;
  if (rev_limit) {
    if (!shouldMove()) {
      // mc_interface_brake_now();
    }
    min_tacho = mc_interface_get_tachometer_value(false);
  }
}

/*
 * Sets the parameter to the provided value in the local configuration
 *
 * Does not update the configuration externally
 */
static void setParameter(mc_config config)
{
  switch(config.param)
  {
    /**
     *  This case includes bools and uint8_t - anything that's a single byte
     */
    case PWM_MODE:
      mcconf.pwm_mode = config.value_byte;
      break;
    case COMM_MODE:
      mcconf.comm_mode = config.value_byte;
      break;
    case MOTOR_TYPE:
      mcconf.motor_type = config.value_byte;
      break;
    case SENSOR_MODE:
      mcconf.sensor_mode = config.value_byte;
      break;
    case L_SLOW_ABS_CURRENT:
      mcconf.l_slow_abs_current = config.value_byte;
      break;
    case ENCODER_INVERTED:
      mcconf.foc_encoder_inverted = config.value_byte;
      break;
    case FOC_SAMPLE_V0_V7:
      mcconf.foc_sample_v0_v7 = config.value_byte;
      break;
    case FOC_SAMPLE_HIGH_CURRENT:
      mcconf.foc_sample_high_current = config.value_byte;
      break;
    case FOC_TEMP_COMP:
      mcconf.foc_temp_comp = config.value_byte;
      break;
    case FOC_SENSOR_MODE:
      mcconf.foc_sensor_mode = config.value_byte;
      break;
    case PID_ALLOW_BRAKING:
      mcconf.s_pid_allow_braking = config.value_byte;
      break;
    case M_SENSOR_PORT_MODE:
      mcconf.m_sensor_port_mode = config.value_byte;
      break;
    case M_DRV8301_OC_MODE:
      mcconf.m_drv8301_oc_mode = config.value_byte;
      break;
    case M_INVERT_DIRECTION:
      mcconf.m_invert_direction = config.value_byte;
      break;

    /**
     *  32-bit signed fields
     */
    case M_FAULT_STOP_TIME_MS:
        mcconf.m_fault_stop_time_ms = config.value_i;
        break;
      case M_DRV8301_OC_ADJ:
        mcconf.m_drv8301_oc_adj = config.value_i;
        break;

    /**
     *  32-bit unsigned fields
     */
    case M_ENCODER_COUNTS:
      mcconf.m_encoder_counts = config.value_ui;
      break;

    // default - the rest are floats
    default:
      *getParamPtr(config.param) = config.value_f;
      break;
    }

  }

  /*
   * Returns a pointer to the parameter in the local configuration
   */
  volatile float *getParamPtr(enum mc_config_param param)
  {
    switch (param) {
      case L_CURRENT_MAX:
        return &mcconf.l_current_max;
      case OBSERVER_GAIN_SLOW:
        return &mcconf.foc_observer_gain_slow;
      case FOC_PLL_KP:
        return &mcconf.foc_pll_kp;
      case SL_CYCLE_INT_RPM:
        return &mcconf.sl_cycle_int_rpm_br;
      case L_IN_CURRENT_MAX:
        return &mcconf.l_in_current_max;
      case OPENLOOP_HYST:
        return &mcconf.foc_sl_openloop_hyst;
      case ENCODER_OFFSET:
        return &mcconf.foc_encoder_offset;
      // case MOTOR_QUALITY_BEARINGS:
      //   return &mcconf.motor_quality_bearings;
      case L_TEMP_FET_END:
        return &mcconf.l_temp_fet_end;
      // case MOTOR_QUALITY_MAGNETS:
      //   return &mcconf.motor_quality_magnets;
      case OPENLOOP_TIME:
        return &mcconf.foc_sl_openloop_time;
      case SL_MIN_ERPM:
        return &mcconf.sl_min_erpm;
      case BATTERY_CUT_END:
        return &mcconf.l_battery_cut_end;
      case FOC_OBSERVER_GAIN:
        return &mcconf.foc_observer_gain;
      case WATT_MAX:
        return &mcconf.l_watt_max;
      case SL_CYCLE_INT_LIMIT:
        return &mcconf.sl_cycle_int_limit;
      case FOC_CURRENT_KI:
        return &mcconf.foc_current_ki;
      case MAX_EPRM_FBRAKE_CC:
        return &mcconf.l_max_erpm_fbrake_cc;
      case MIN_DUTY:
        return &mcconf.l_min_duty;
      case FOC_CURRENT_KP:
        return &mcconf.foc_current_kp;
    case DUTY_RAMP_STEP:
      return &mcconf.m_duty_ramp_step;
    case ABS_CURRENT_MAX:
      return &mcconf.l_abs_current_max;
    case S_PID_MIN_ERPM:
      return &mcconf.s_pid_min_erpm;
    case M_NTC_MOTOR_BETA:
      return &mcconf.m_ntc_motor_beta;
    case MAX_DUTY:
      return &mcconf.l_max_duty;
    case MOTOR_FLUX_LINKAGE:
      return &mcconf.foc_motor_flux_linkage;
    case MAX_ERPM_FBRAKE:
      return &mcconf.l_max_erpm_fbrake;
    case HALL_SL_ERPM:
      return &mcconf.hall_sl_erpm;
    case FOC_SL_D_CURRENT_FACTOR:
      return &mcconf.foc_sl_d_current_factor;
    case CC_MIN_CURRENT:
      return &mcconf.cc_min_current;
    case CC_RAMP_STEP_MAX:
      return &mcconf.cc_ramp_step_max;
    case L_ERPM_START:
      return &mcconf.l_erpm_start;
    case FOC_DT_US:
      return &mcconf.foc_dt_us;
    case CC_GAIN:
      return &mcconf.cc_gain;
    case L_MAX_VIN:
      return &mcconf.l_max_vin;
    case M_BLDC_F_SW_MIN:
      return &mcconf.m_bldc_f_sw_min;
    // case MOTOR_WEIGHT:
    //   return &mcconf.motor_weight;
    case P_PID_KD:
      return &mcconf.p_pid_kd;
    case L_TEMP_MOTOR_END:
      return &mcconf.l_temp_motor_end;
    case FOC_SAT_COMP:
      return &mcconf.foc_sat_comp;
    case P_PID_ANG_DIV:
      return &mcconf.p_pid_ang_div;
    case MAX_FULLBREAK_CURRENT_DIR_CHANGE:
      return &mcconf.sl_max_fullbreak_current_dir_change;
    case L_BATTERY_CUT_START:
      return &mcconf.l_battery_cut_start;
    case P_PID_KI:
      return &mcconf.p_pid_ki;
    case FOC_SL_D_CURRENT_DUTY:
      return &mcconf.foc_sl_d_current_duty;
    case CC_STARTUP_BOOST_DUTY:
      return &mcconf.cc_startup_boost_duty;
    case FOC_TEMP_COMP_BASE_TEMP:
      return &mcconf.foc_temp_comp_base_temp;
    case P_PID_KP:
      return &mcconf.p_pid_kp;
    case L_MIN_ERPM:
      return &mcconf.l_min_erpm;
    case FOC_SL_ERPM:
      return &mcconf.foc_sl_erpm;
    // case MOTOR_BRAND:
    //   return &mcconf.motor_brand;
    case M_BLDC_F_SW_MAX:
      return &mcconf.m_bldc_f_sw_max;
    case S_PID_KD:
      return &mcconf.s_pid_kd;
    case L_TEMP_FET_START:
      return &mcconf.l_temp_fet_start;
    case L_MAX_ERPM:
      return &mcconf.l_max_erpm;
    case SL_PHASE_ADVANCE_AT_BR:
      return &mcconf.sl_phase_advance_at_br;
    case FOC_MOTOR_L:
      return &mcconf.foc_motor_l;
    case S_PID_KI:
      return &mcconf.s_pid_ki;
    case L_MIN_VIN:
      return &mcconf.l_min_vin;
    case FOC_MOTOR_R:
      return &mcconf.foc_motor_r;
    case FOC_DUTY_DOWNRAMP_KI:
      return &mcconf.foc_duty_dowmramp_ki;
    // case MOTOR_QUALITY_CONSTRUCTION:
    //   return &mcconf.motor_quality_construction;
    case S_PID_KP:
      return &mcconf.s_pid_kp;
    case FOC_DUTY_DOWNRAMP_KP:
      return &mcconf.foc_duty_dowmramp_kp;
    case L_CURRENT_MIN:
      return &mcconf.l_current_min;
    case FOC_ENCODER_RATIO:
      return &mcconf.foc_encoder_ratio;
    case L_IN_CURRENT_MIN:
      return &mcconf.l_in_current_min;
    case M_CURRENT_BACKOFF_GAIN:
      return &mcconf.m_current_backoff_gain;
    case SL_BEMF_COUPLING_K:
      return &mcconf.sl_bemf_coupling_k;
    // case MOTOR_SENSOR_TYPE: 
    //   return &mcconf.sensor_mode;
    // case MOTOR_MODEL:
    //   return &mcconf.motor_model;
    case SL_MIN_ERPM_CYCLE_INT_LIMIT:
      return &mcconf.sl_min_erpm_cycle_int_limit;
    case L_TEMP_MOTOR_START:
      return &mcconf.l_temp_motor_start;
    case OPENLOOP_RPM:
      return &mcconf.foc_openloop_rpm;
    // case MOTOR_LOSS_TORQUE:
    //   return &mcconf.motor_loss_torque;
    case M_DC_F_SW:
      return &mcconf.m_dc_f_sw;
    case FOC_F_SW:
      return &mcconf.foc_f_sw;
    // case MOTOR_POLES:
    //   return &mcconf.motor_poles;
    case FOC_PLL_KI: 
      return &mcconf.foc_pll_ki;
    case L_WATT_MIN:
      return &mcconf.l_watt_min;
    default:
      return NULL;
  }
}

static int getStringPotValue()
{
  return ADC_Value[7];
}
