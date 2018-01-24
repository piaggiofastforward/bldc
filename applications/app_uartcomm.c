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
// #include "ext_lld.h"
#include "uart_mc_config.h"
#include "encoder.h"

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
volatile bool commitConfigReceived = false;
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
static void detectHallTableFoc(void);
static void setHall(hall_table_t hall_table);
static void setHallFoc(hall_table_foc_t hall_table);
static void setCommand(void);
static void sendConfigCommitConfirmation(void);
static int getStringPotValue(void);
volatile float *getParamPtr(enum mc_config_param param);
int32_t descale_position(float pos);
bool shouldMove(void);
void homing_sequence(void);
// void toggle_estop(EXTDriver *extp, expchannel_t channel); 
// void toggle_fwd_limit(EXTDriver *extp, expchannel_t channel);
// void toggle_rev_limit(EXTDriver *extp, expchannel_t channel);


/**
 * Use timer interrupts to trigger status and feedback publishing at specified intervals. Set flags in the
 * interrupts, handle in the main thread.
 */
static virtual_timer_t feedback_task_vt;
static virtual_timer_t status_task_vt;
#define isPublishing() (chVTIsArmedI(&feedback_task_vt) || chVTIsArmedI(&status_task_vt))

/**
 *  This timer will be used to trigger a task that reads the hall sensors and sends the data
 *  over UART.
 */
static virtual_timer_t hall_task_vt;
#define HALL_READ_RATE_MS 10
#define HALL_PACKET_HEADER 47
static uint8_t hall_data[3] = {};
static bool shouldSendHallData = false;

static void hallReadCb(void* _);
static void feedbackTaskCb(void* _);
static void statusTaskCb(void* _);
void sendHallData(void);
void sendFeedback(void);
void updateFeedback(void);
void sendStatus(void);
void updateStatus(void);

/**
 *  Enable/disable the virtual timer interrupts that prompt us to send feedback/status data. This 
 *  will be used when we get an FOC detection request.
 */
static void disablePublishing(void);
static void enablePublishing(void);

/**
 *  Use this to aid in stopping/starting publishing when we receive a CONFIG_WRITE command.
 *  Schedule publishing to start up again in the amount of time below.
 */
static virtual_timer_t start_pub_task_vt;
static void startPubTaskCb(void* _);
#define DELAY_CONFIG_WRITE_START_PUB_MS 50

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
 *  Define a timer task to get the absolute encoder ticks at a predefined rate
 */
#define GET_ENC_TICKS_RATE_MS 5
static virtual_timer_t get_encoder_ticks_task;
static volatile enc_abs_count_t encoder_abs_ticks = 0;
static volatile bool shouldReadEncTicks = false;
static void getTicksCb(void* _);

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
#define ESTOP_PIN_INDEX  5
#define FWDLIM_PIN_INDEX 3
#define REVLIM_PIN_INDEX 4
#define ESTOP_PORT       GPIOC
#define FWDLIM_PORT      GPIOA
#define REVLIM_PORT      GPIOA

// static const EXTConfig extcfg = {
//   {
//     {EXT_CH_MODE_DISABLED, NULL},    
//     {EXT_CH_MODE_DISABLED, NULL},    
//     {EXT_CH_MODE_DISABLED, NULL},    
//     {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, toggle_fwd_limit},
//     {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, toggle_rev_limit},
//     {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, toggle_estop},
//     {EXT_CH_MODE_DISABLED, NULL},   
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL},
//     {EXT_CH_MODE_DISABLED, NULL}
//   }
// };


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
      if (isPublishing())
      {
        disablePublishing();
      }
      // start publishing again if we dont receive a config within 50 ms
      chVTSet(&start_pub_task_vt, MS2ST(DELAY_CONFIG_WRITE_START_PUB_MS), startPubTaskCb, NULL);
      memcpy(config.config_bytes, data + 1, sizeof(mc_config));
      setParameter(config.config, &mcconf);
			// updateConfigReceived = true;
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
        disablePublishing();
      }
      // at this point, dont start publishing until after we send the confirmation response
      chVTReset(&start_pub_task_vt);
      commitConfigReceived = true;
      break;

    case REQUEST_DETECT_HALL_FOC:

      // dont do things (like send feedback and accept commands to drive the motor!!!)
      disablePublishing();
      detectHallTableFoc();
      chThdSleepMilliseconds(200);
      enablePublishing();
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

  // extStart(&EXTD1, &extcfg);
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
   * Initialize timers for feedback and status reports. These timers will set the shouldSendFeedback,
   * shouldSendStatus, and shouldReadEncTicks flags in an interrupt context, and we will do the actual
   * operation in the main thread.
   */
  // chVTObjectInit(&start_pub_task_vt);
  // chVTObjectInit(&status_task_vt);
  // chVTObjectInit(&feedback_task_vt);
  // chVTObjectInit(&get_encoder_ticks_task);
  chVTObjectInit(&hall_task_vt);
  chVTSet(&hall_task_vt, MS2ST(HALL_READ_RATE_MS), hallReadCb, NULL);
  // chVTSet(&feedback_task_vt, MS2ST(FB_RATE_MS), feedbackTaskCb, NULL);
  // chVTSet(&status_task_vt, MS2ST(STATUS_INITIAL_DELAY), statusTaskCb, NULL);
  // chVTSet(&get_encoder_ticks_task, MS2ST(GET_ENC_TICKS_RATE_MS), getTicksCb, NULL);

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

    // while (!uartReceiving && (serial_rx_read_pos != serial_rx_write_pos))
    // {

    //   /**
    //    * This should be called on every received byte. When the packet interface detects the end of
    //    * a packet, it will call its process_func - in our case, this will be process_packet() above. 
    //    */
    //   packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], PACKET_HANDLER);

    //   if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
    //     serial_rx_read_pos = 0;
    //   }
    // }

    // *
    //  * Handle the case where the uartStartReceive function returned (ie: the MAX_BYTES_PER_READ)
    //  * was met) make sure we first process the bytes received from rxChar() (while statement above)
    //  * and then process all of these bytes so that we call packet_process_byte() in the correct sequence.
     

    // if (rxEndReceived)
    // {
    //   for (int i = 0; i < packetLength; i++)
    //   {
    //     packet_process_byte(uart_receive_buffer[i], PACKET_HANDLER);
    //   }
    //   rxEndReceived = false;
    //   uartReceiving = false;
    // }

    if (shouldSendHallData)
    {
      sendHallData();
      shouldSendHallData = false;
    }

  //   /**
  //    * If we received the packetLength, use uartStartReceive() to receive the rest of the bytes. When
  //    * the specified number of bytes is read, rxEnd() will be invoked, setting rxEndReceived = true
  //    */
  //   if (packetLengthReceived)
  //   {
  //     uartStartReceive(&HW_UART_DEV, packetLength, uart_receive_buffer);
  //     uartReceiving = true;
  //     packetLengthReceived = false;
  //   }

  //   /**
  //    * These flags will be set in timer interrupts at rates configurable via FB_RATE_MS and STATUS_RATE_MS
  //    */
		// if (shouldSendFeedback)
  //   {
  //     sendFeedback();
  //     shouldSendFeedback = false;
  //   }
  //   if (shouldSendStatus)
  //   {
  //     sendStatus();
  //     shouldSendStatus = false;
  //   }

  //   /**
  //    * TODO: proper handling of estop (brake? or just set command to 0?)
  //    */
		// if (estop)
		// {
  //     mcpwm_foc_stop_pwm();
		// 	// mc_interface_set_brake_current(0);
		// }
		// else if (!shouldMove())
		// {
		// 	// mc_interface_brake_now();
		// }

  //   else if (commandReceived)
		// {
		// 	setCommand();
		// 	commandReceived = false;
		// }

  //   if (commitConfigReceived)
  //   {
  //     disablePublishing();
  //     conf_general_store_mc_configuration((mc_configuration *) &mcconf);
  //     mc_interface_set_configuration((mc_configuration *) &mcconf);
  //     chThdSleepMilliseconds(500);
  //     sendConfigCommitConfirmation();
  //     commitConfigReceived = false;
  //     enablePublishing();
  //   }
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

static void hallReadCb(void* _)
{
  (void)_;
  chSysLockFromISR();

  hall_data[0] = READ_HALL1();
  hall_data[1] = READ_HALL2();
  hall_data[2] = READ_HALL3();

  chVTSetI(&hall_task_vt, MS2ST(HALL_READ_RATE_MS), hallReadCb, NULL);
  shouldSendHallData = true;
  chEvtSignalI(process_tp, (eventmask_t) 1);
  chSysUnlockFromISR();
}

void sendHallData()
{
  uint8_t data[4] = {
    HALL_PACKET_HEADER,
    hall_data[0],
    hall_data[1],
    hall_data[2],
  };
  send_packet_wrapper(data, 4);
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
static void disablePublishing(void)
{
  chVTReset(&feedback_task_vt);
  chVTReset(&status_task_vt);
}

static void enablePublishing(void)
{
  chVTSetI(&feedback_task_vt, MS2ST(FB_RATE_MS), feedbackTaskCb, NULL);
  chVTSetI(&status_task_vt, MS2ST(STATUS_RATE_MS), statusTaskCb, NULL);
}

/**
 * Set flags to indicate that we should publish feedback or status, handle in main thread.
 */
static void startPubTaskCb(void* _)
{
  (void)_;
  chSysLockFromISR();
  enablePublishing();
  chSysUnlockFromISR();
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

static void getTicksCb(void* _)
{
	(void) _;
	shouldReadEncTicks = true;
	chSysLockFromISR();
	encoder_abs_ticks = encoder_abs_count();
  chVTSetI(&get_encoder_ticks_task, MS2ST(GET_ENC_TICKS_RATE_MS), getTicksCb, NULL);
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
  // fb.feedback.measured_position = mc_interface_get_pid_pos_now();
  fb.feedback.measured_position = encoder_abs_count();
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
      echoCommand();
      // mc_interface_set_pid_speed(currentCommand.target_cmd_f);
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

// /*
//  * Reads the estop value and disables control if it is set
//  * Called from interrupt context on both edges
//  * Pulldown input, so unless externally pulled up, default value is PAL_LOW
//  */
// void toggle_estop(EXTDriver *extp, expchannel_t channel) 
// {
//   (void) extp;
//   (void) channel;
//   estop = palReadPad(ESTOP_PORT, ESTOP_PIN_INDEX) == PAL_LOW;
//   // if (estop) {
//   //   mc_interface_set_brake_current(0);
//   // }
// }

// /*
//  * Reads the forward limit switch. If set and trying to move forward, brake
//  * Sets the max_tacho limit
//  * Called from interrupt context on both edges
//  */
// void toggle_fwd_limit(EXTDriver *extp, expchannel_t channel) 
// {
//   (void) extp;
//   (void) channel;
//   fwd_limit = palReadPad(FWDLIM_PORT, FWDLIM_PIN_INDEX) == PAL_LOW;
//   if (fwd_limit) {
//     if (!shouldMove()) {
//       // mc_interface_brake_now();
//     }
//     max_tacho = mc_interface_get_tachometer_value(false);
//   }
// }

// /*
//  * Reads the reverse limit switch. If set and trying to move backwards, brake
//  * Sets the min_tacho limit
//  * Called from interrupt context on both edges
//  */
// void toggle_rev_limit(EXTDriver *extp, expchannel_t channel) 
// {
//   (void) extp;
//   (void) channel;
//   rev_limit = palReadPad(REVLIM_PORT, REVLIM_PIN_INDEX) == PAL_LOW;
//   if (rev_limit) {
//     if (!shouldMove()) {
//       // mc_interface_brake_now();
//     }
//     min_tacho = mc_interface_get_tachometer_value(false);
//   }
// }

static int getStringPotValue()
{
  return ADC_Value[7];
}
