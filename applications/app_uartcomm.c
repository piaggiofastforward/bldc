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

// Settings
#define BAUDRATE					115200
#define PACKET_HANDLER				1
#define SERIAL_RX_BUFFER_SIZE		1024

// Threads
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);
static thread_t *process_tp;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static unsigned int serial_rx_read_pos = 0;
static unsigned int serial_rx_write_pos = 0;
static unsigned int MAX_PACKET_LENGTH = 15;
static int is_running = 0;

// functions that work with the packet interface
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);


// state variables
volatile bool rxEndReceived        = false;
volatile bool rxCharReceived       = false;
volatile bool commandReceived      = false;
volatile bool updateConfigReceived = false;
static volatile int32_t min_tacho = INT_MAX;
static volatile int32_t max_tacho = INT_MIN;
static volatile bool estop = true;
static volatile bool rev_limit = false;
static volatile bool fwd_limit = false;


// structs to hold transaction data
static volatile mc_feedback_union fb;
static volatile mc_status_union status;
static volatile mc_request_union request;
static volatile mc_request currentCommand = {{0}, {0}, 0};
static volatile mc_configuration mcconf;

// forward declarations of handler functions
static void setHall(hall_table_t hall_table);
static void setCommand(void);
static void setParameter(enum mc_config_param param, float value);
static int getStringPotValue(void);

// feedback and status publishing functions, as well as virtual timers for
// executing these tasks
static virtual_timer_t feedback_task_vt;
static virtual_timer_t status_task_vt;

// use so that we dont publish feedback and status simultaneously
static void feedbackTaskCb(void* _);
static void statusTaskCb(void* _);

void sendFeedback(void);
void updateFeedback(void);
void sendStatus(void);
void updateStatus(void);


// for testing purposes
void echoCommand();
void confirmationEcho();


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

volatile float *getParamPtr(enum mc_config_param param);

int32_t descale_position(float pos);
bool shouldMove(void);
void homing_sequence(void);

void toggle_estop(EXTDriver *extp, expchannel_t channel); 
void toggle_fwd_limit(EXTDriver *extp, expchannel_t channel);
void toggle_rev_limit(EXTDriver *extp, expchannel_t channel);


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
static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, toggle_estop},
    {EXT_CH_MODE_DISABLED, NULL},    
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
 * If this happens, we want to prepare the driver to properly receive the rest
 * of the message.
 */
static void rxchar(UARTDriver *uartp, uint16_t c)
{
	(void)uartp;
	serial_rx_buffer[serial_rx_write_pos++] = c;
	rxCharReceived = true;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chSysLockFromISR();

	// add event flags to process_tp, probably so we can exit the chEventAwaitAny call...?
	chEvtSignalI(process_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/*
 * This callback is invoked when the receive buffer is
 * completely filled!!!
 */
static void rxend(UARTDriver *uartp)
{
	(void)uartp;
	rxEndReceived = true;
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
  memcpy(request.request_bytes, data + 1, sizeof(mc_request));

	// // switch (data[0])
 //  switch (request.request.type)
	// {
	// 	case CONFIG_READ:

	// 		// need to implement
	// 		break;

	// 	case CONTROL_WRITE:
 //      // echoCommand();
	// 		currentCommand = request.request;
	// 		commandReceived = true;
	// 		timeout_reset();
	// 		break;

	// 	case CONFIG_WRITE:
	// 		if (request.request.param == HALL_TABLE)
	// 			setHall(request.request.value_hall);
	// 		else
	// 			setParameter(request.request.param, request.request.value_f);
	// 		updateConfigReceived = true;
	// 		break;

	// 	default:
	// 		break;
	// }
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

void app_uartcomm_start(void)
{
	packet_init(send_packet, process_packet, PACKET_HANDLER);

	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);

	extStart(&EXTD1, &extcfg);
	estop = palReadPad(GPIOA, 5) == PAL_LOW;
  rev_limit = palReadPad(GPIOA, 6) == PAL_LOW;
	rev_limit = false;
  fwd_limit = palReadPad(GPIOC, 0) == PAL_LOW;
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
   * Initialize timers for feedback and status reports. Start publishing status a 
   * little later so that we dont try to publish status and feedback at the same time.
   */
  // chVTObjectInit(&status_task_vt);
  // chVTObjectInit(&feedback_task_vt);
  // chVTSet(&feedback_task_vt, MS2ST(FB_RATE_MS), feedbackTaskCb, NULL);
  // chVTSet(&status_task_vt, MS2ST(STATUS_INITIAL_DELAY), statusTaskCb, NULL);

	while (1)
	{
		// chEvtWaitAny((eventmask_t) 1);

		// send out feedback on every loop. If we are not receiving data, this will happen at about
		// 50Hz
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

		if (estop)
		{
			// mc_interface_set_brake_current(0);
		}
		else if (!shouldMove())
		{
			// mc_interface_brake_now();
		}
		if (commandReceived)
		{
			setCommand();

      // for testing purposes, just write the command back over UART
      // echoCommand();
			commandReceived = false;
		}

		if (updateConfigReceived)
		{
			// do stuff
			mc_interface_set_configuration((mc_configuration *) &mcconf);
			conf_general_store_mc_configuration((mc_configuration *) &mcconf);
			updateConfigReceived = false;
		}

		/**
		 * We can receive data in 2 ways:
		 * 
		 * 1) Through the rxchar() callback above. This will be called when we receive a character
		 *    but the application wasnt ready for it. In this case, we want to prepare the UARTDriver
		 *    object to correctly receive the rest of the packet, placing all the data in the
		 *		serial_rx_buffer.
		 *
		 * 2) Through calls to uartReceive(). This will ensure a "proper" receiving of data - 
		 *    the rxchar() callback will not be invoked, but the data will still be placed into
		 *    the serial_rx_buffer.
		 */

		// not sure that we want to go this route. if we let the rxend() callback occur on every char transfer,
		// that will continually have us call packet_process_byte() until a full request is received.
 		//
		// if we receive some chars in the serial_rx_buffer through rxend() and then try to receive the rest
		// through uartStartReceive(), some (probably just 1) chars will end up in the serial_rx_buffer, 
		// and the rest of the chars will end up in whatever buffer we pass to the function....


		if (rxCharReceived)
		{
			uartStartReceive(&HW_UART_DEV, MAX_PACKET_LENGTH, serial_rx_buffer);
			rxCharReceived = false;
		}

		// check to see if we received a full message...do we need to do anything in that situation?
		if (rxEndReceived)
		{
      confirmationEcho();
      rxEndReceived = false;
		}
		
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

    chThdSleepMilliseconds(1);
	}
}



/*****************************************************************************
	
													Implementation-specific

*****************************************************************************/

// for testing purposes, you can have the VESC echo back a received request
// instead of actually issuing the command
void echoCommand()
{
  uint8_t data[sizeof(mc_request) + 1];
  data[0] = CONTROL_WRITE;
  memcpy(data + 1, request.request_bytes, sizeof(mc_request));
  send_packet_wrapper(data, sizeof(data)); 
}

void confirmationEcho()
{
  uint8_t confirmationBuf[3] = {0, 0, 0};
  send_packet_wrapper(confirmationBuf, 3);
}

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
  // fb.feedback.switch_flags      = ST2MS(chVTGetSystemTimeX());
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
  switch (currentCommand.control) {
    case SPEED:
      fb.feedback.commanded_value = currentCommand.value_i;
      echoCommand();
      mc_interface_set_pid_speed(currentCommand.value_i);
      break;
    // case CURRENT:
    //   fb.feedback.commanded_value = currentCommand.value_f * 1000; 
    //   mc_interface_set_current(currentCommand.value_f);
    //   break;
    // case DUTY:
    //   fb.feedback.commanded_value = currentCommand.value_f * 1000;
    //   mc_interface_set_duty(currentCommand.value_f);
    //   break;
    // case POSITION:
    //   fb.feedback.commanded_value = currentCommand.value_i;
    //   mc_interface_set_pid_pos(currentCommand.value_i);
    //   break;
    // case SCALE_POS:
    //   fb.feedback.commanded_value = currentCommand.value_f * 1000;
    //   mc_interface_set_pid_pos(descale_position(currentCommand.value_f));
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
  memcpy((void *) mcconf.foc_hall_table, hall_table, HALL_TABLE_SIZE);
}

/*
 * Sets the parameter to the provided value in the local configuration
 *
 * Does not update the configuration externally
 */
static void setParameter(enum mc_config_param param, float value)
{
  *getParamPtr(param) = value;
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
  switch (currentCommand.control) {
    case POSITION:
      isForward = currentCommand.value_i - mc_interface_get_tachometer_value(false) > 0;
      break;
    case SCALE_POS:
      isForward = descale_position(currentCommand.value_f / 1000.0f) > 0 - 
        mc_interface_get_tachometer_value(false);
      break;
    case SPEED:
      isForward = currentCommand.value_i > 0;
      break;
    case HOMING:
      if (max_tacho == INT_MIN) {
        isForward = true;
      } else if (min_tacho == INT_MAX) {
        isForward = false;
      }
    case CURRENT:
    case DUTY:
      isForward = currentCommand.value_f > 0;
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
  estop = palReadPad(GPIOA, 5) == PAL_LOW;
  if (estop) {
    mc_interface_set_brake_current(0);
  }
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
  fwd_limit = palReadPad(GPIOC, 0) == PAL_HIGH;
  if (fwd_limit) {
    if (!shouldMove()) {
      mc_interface_brake_now();
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
  rev_limit = palReadPad(GPIOA, 6) == PAL_HIGH;
  if (rev_limit) {
    if (!shouldMove()) {
      mc_interface_brake_now();
    }
    min_tacho = mc_interface_get_tachometer_value(false);
  }
}

/*
 * Returns a pointer to the parameter in the local configuration
 */
volatile float *getParamPtr(enum mc_config_param param)
{
  switch (param) {
    case FOC_KP:
      return &mcconf.foc_current_kp;
    case FOC_KI:
      return &mcconf.foc_current_ki;
    case MOTOR_L:
      return &mcconf.foc_motor_l;
    case MOTOR_R:
      return &mcconf.foc_motor_r;
    case MOTOR_FLUX:
      return &mcconf.foc_motor_flux_linkage;
    case OBSERVER_GAIN:
      return &mcconf.foc_observer_gain;
    case MAX_CURRENT:
      return &mcconf.l_current_max;
    case MIN_CURRENT:
      return &mcconf.l_current_min;
    case POS_PID_KP:
      return &mcconf.p_pid_kp;
    case POS_PID_KI:
      return &mcconf.p_pid_ki;
    case POS_PID_KD:
      return &mcconf.p_pid_kd;
    case SPEED_PID_KP:
      return &mcconf.s_pid_kp;
    case SPEED_PID_KI:
      return &mcconf.s_pid_ki;
    case SPEED_PID_KD:
      return &mcconf.s_pid_kd;
    case SPEED_PID_MIN_RPM:
      return &mcconf.s_pid_min_erpm;
    default:
      return NULL;
  }
}

static int getStringPotValue()
{
  return ADC_Value[7];
}