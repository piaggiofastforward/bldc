/* 
 * app_i2cslave.c 
 * Aubrey Anderson, 18 July 2017 
 * Piaggio Fast Forward 
 * This defines an application within the VESC framework that controls the motor
 * controller using I2C slave functionality.  The controller must receive a command at
 * least once per second to reset the watchdog timer.  Set the I2C address for the
 * controller using the controller ID field in the BLDC tool
 */

/* MESSAGES 
 * To read feedback: 
 *   Read from the I2C Address 
 *     Format: 
 *       Each field in mc_feedback struct separated by colons as ASCII 
 *       Floats are rounded at 3 decimal places 
 * To read status: 
 *   Write the byte VESC_STATUS to the I2C Address 
 *   Read from the I2C Address 
 *   Format: 
 *     FAULT_CODE:TEMP 
 *     Each field is ASCII 
 *     TEMP is rounded at 3 decimal places 
 *   To send command: 
 *     Write the byte desired VESC_CONTROL type to the I2C Address followed by the control value.  
 *     Format: CONTROL_VAL
 *     CONTROL_VAL is ASCII
 */
#include <stdio.h> // For snprintf 
#include <stdlib.h> // For atof
#include <math.h>
#include <limits.h>
#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "mcpwm_foc.h" // Motor control functions
#include "hw.h" // HW Pin Mapping
#include "timeout.h" // Reset MC Timeout
#include "i2cslave.h"
#include "i2c_lld.h"
#include "ext_lld.h"
#include "pal.h"
#include "applications/i2c_control_msgs.h"

// Function prototypes
void requestProcessor(I2CDriver *i2cp);
void catchError(I2CDriver *i2cp);
void createFeedbackReply(I2CDriver *i2cp);
void feedbackReplyDone(I2CDriver *i2cp);
void statusReplyDone(I2CDriver *i2cp);
void updateStatus(I2CDriver *i2cp);
void setCommand(void);
void noteI2cError(uint32_t flags);
void updateFeedback(void);
void app_i2cslave_init(void);
int  descale_position(float pos);
void reset_homing(void);

// Thread for VESC application
static THD_FUNCTION(i2cslave_thread, arg);
static THD_WORKING_AREA(i2cslave_thread_wa, 2048);

// Feedback struct to store feedback info between requests
typedef struct {
  int32_t motor_current;
  int32_t commanded_velocity;
  int32_t measured_velocity;
  int32_t measured_position;
  int32_t supply_voltage;
  int32_t supply_current;
  uint32_t switch_flags;
} mc_feedback;

typedef union {
  mc_feedback feedback;
  uint8_t feedback_bytes[sizeof(mc_feedback)];
} mc_feedback_union;

typedef struct {
  int32_t value;
  uint8_t type;
} mc_command;

union {
  mc_command request;
  uint8_t request_bytes[sizeof(mc_command)];
} request;

typedef struct {
  int32_t fault_code;
  int32_t temp;
} mc_status;

typedef union {
  mc_status status;
  uint8_t status_bytes[sizeof(mc_status)];
} mc_status_union;

static volatile mc_command command_now = {0, 0};

static mc_feedback_union fb;
static mc_status_union status;
static volatile bool needFeeback = false;
static volatile bool gotCommand = false;
static I2CDriver *i2cp_g = NULL;
static i2caddr_t mc_i2c_addr = 0x00; // default I2C Address, by design will crash the app
static bool enable_homing = false;

// Handler for write messages sent to motor controller
static const I2CSlaveMsg requestRx = {
  sizeof(request.request_bytes),       // max sizeof received msg body 
  request.request_bytes,               // body of received msg
  NULL,                      // do nothing on address match
  requestProcessor,          // Routine to process received messages
  catchError                 // Error hook
};

// Default response to read messages
static I2CSlaveMsg feedbackReply = {  // this is in RAM so size may be updated
  0,                           // Zero so clock will stretch until response is ready
  fb.feedback_bytes,                // Feedback buffer
  createFeedbackReply,         // Convert feedback struct to string
  feedbackReplyDone,           // Clear feedback size once replied
  catchError                   // Error hook
};

// Response after VESC_STATUS is written
static I2CSlaveMsg statusReply = {  // this is in RAM so size may be updated
  0,                         // Zero so clock will stretch until response is ready 
  status.status_bytes,                // Status buffer
  updateStatus,              // Create status message
  statusReplyDone,           // Reset to default feedback reply
  catchError                 // Error hook
};

// I2C error variables
uint8_t gotI2cError = 0;
uint32_t lastI2cErrorFlags = 0;

// Called from ISR to log error
void noteI2cError(uint32_t flags)
{
  lastI2cErrorFlags = flags;
  gotI2cError = 1;
}

/**
 * Generic error handler
 *
 * Called in interrupt context, so need to watch what we do
 */
void catchError(I2CDriver *i2cp)
{
  noteI2cError(i2cp->errors);
}

/*
 * Callback for received messages
 * Handles either setting a command or changing the response for the next read
 * Called in interrupt context
 */
void requestProcessor(I2CDriver *i2cp)
{ 
  switch (request.request.type) {
    case VESC_CONTROL_SPEED:
    case VESC_CONTROL_POSITION:
    case VESC_CONTROL_CURRENT:
    case VESC_CONTROL_DUTY:
    case VESC_CONTROL_SCALE_POS:
      command_now = request.request;
      gotCommand = true;
      timeout_reset();
      break;
    case VESC_FEEDBACK:
      i2cSlaveReplyI(i2cp, &feedbackReply);
      break;
    case VESC_STATUS:
      i2cSlaveReplyI(i2cp, &statusReply);
      break;
    case VESC_CONTROL_HOMING:
      reset_homing();
      enable_homing = true;
      break;
    default:
      break;
  }
}

/*
 * Sets the command based on the control mode
 *
 * Resets the watchdog timer
 * Called in interrupt context
 */
void setCommand()
{
  /* int error; */
  /* static int error_integral = 0; */
  switch (command_now.type) {
    fb.feedback.commanded_velocity = command_now.value;
    case VESC_CONTROL_SPEED:
      mc_interface_set_pid_speed(command_now.value);
      break;
    case VESC_CONTROL_CURRENT:
      mc_interface_set_current(command_now.value / 1000);
      break;
    case VESC_CONTROL_DUTY:
      mc_interface_set_duty(command_now.value / 1000);
      break;
    case VESC_CONTROL_POSITION:
      mc_interface_set_pid_pos(command_now.value);
      break;
    case VESC_CONTROL_SCALE_POS:
      mc_interface_set_pid_pos(descale_position(command_now.value / 1000.0));
      break;
    default:
      break;
  }
}

/*
 * Updates the feedback buffer with data from the feedback struct
 * Calls i2cSlaveReply to re-enable I2C clocking once message is ready
 * Called in interrupt context
 */
void createFeedbackReply(I2CDriver *i2cp)
{
  needFeeback = true;
  i2cp_g = i2cp;
}

/*
 * Updates the status buffer with current status data
 * Calls i2cSlaveReply to re-enable I2C clocking once message is ready
 * Called in interrupt context
 */
void updateStatus(I2CDriver *i2cp)
{
  status.status.fault_code = mc_interface_get_fault();
  status.status.temp = NTC_TEMP(ADC_IND_TEMP_MOS2);
  statusReply.size = sizeof(mc_status);
  i2cSlaveReplyI(i2cp, &statusReply);
}

/*
 * Callback for when feedbackReply is done sending
 * Resets size of reply so clock stretching will occur on next read
 * Called in interrupt context
 */
void feedbackReplyDone(I2CDriver *i2cp)
{
  (void) i2cp;
  needFeeback = false;
  feedbackReply.size = 0;
}

/*
 * Callback for when feedbackReply is done sending
 *
 * Resets size of reply so clock stretching will occur on next status read
 * Resets to default feedback reply
 * Called in interrupt context
 */
void statusReplyDone(I2CDriver *i2cp)
{
  statusReply.size = 0;
  i2cSlaveReplyI(i2cp, &feedbackReply);
}


static bool estop = true;
static bool rev_limit = INT_MAX;
static bool fwd_limit = INT_MIN;
/*
 * Updates the feedback struct with current data
 */ void updateFeedback()
{
  fb.feedback.motor_current = mc_interface_get_tot_current() * 1000;
  fb.feedback.measured_velocity = mc_interface_get_rpm();
  fb.feedback.measured_position = mc_interface_get_tachometer_value(false);
  fb.feedback.supply_voltage  = GET_INPUT_VOLTAGE() * 1000;
  fb.feedback.supply_current = mc_interface_get_tot_current_in() * 1000;
  fb.feedback.switch_flags = (estop << 2) | (rev_limit << 1) | (fwd_limit);
}

int32_t min_tacho = INT_MAX;
int32_t max_tacho = INT_MIN;

void reset_homing() {
  min_tacho = INT_MAX;
  max_tacho = INT_MIN;
}

void homing_sequence(void) {
  if (max_tacho == INT_MIN) {
    if (!fwd_limit) {
      mc_interface_set_current(2);
    } 
  } else if (min_tacho == INT_MAX) {
    if (!rev_limit) {
      mc_interface_set_current(-2);
    }
  } else {
    enable_homing = false;
  }
}


void toggle_estop(EXTDriver *extp, expchannel_t channel) {
  (void) extp;
  (void) channel;
  estop = palReadPad(GPIOA, 5) == PAL_LOW;
  if (estop) {
    mc_interface_set_brake_current(0);
  }
}

void toggle_fwd_limit(EXTDriver *extp, expchannel_t channel) 
{
  (void) extp;
  (void) channel;
  fwd_limit = palReadPad(GPIOC, 0) == PAL_LOW;
  if (fwd_limit) {
    if (mc_interface_get_duty_cycle_now() > 0) {
      mc_interface_brake_now();
    }
    max_tacho = mc_interface_get_tachometer_value(false);
  }
}

void toggle_rev_limit(EXTDriver *extp, expchannel_t channel) 
{
  (void) extp;
  (void) channel;
  rev_limit = palReadPad(GPIOA, 6) == PAL_HIGH;
  if (rev_limit) {
    if (mc_interface_get_duty_cycle_now() < 0) {
      mc_interface_brake_now();
    }
    min_tacho = mc_interface_get_tachometer_value(false);
  }
}

int descale_position(float pos)
{
  /* assert(pos >= -1 && pos <= 1); */
    int offset = (max_tacho - min_tacho) * pos / 2;
    int center = (max_tacho + min_tacho) / 2;
    return center + offset;
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, toggle_fwd_limit},
    {EXT_CH_MODE_DISABLED, NULL},    
    {EXT_CH_MODE_DISABLED, NULL},    
    {EXT_CH_MODE_DISABLED, NULL},   
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, toggle_estop},
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
 * Initialization for the i2c slave control
 * Starts the I2C bus and configures it to respond 
 */
void app_i2cslave_init() 
{
  hw_start_i2c();

  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT);
  palSetPadMode(GPIOA, 6, PAL_MODE_INPUT);
  palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_PULLUP);
  extStart(&EXTD1, &extcfg);

  estop = palReadPad(GPIOA, 5) == PAL_LOW;
  rev_limit = palReadPad(GPIOC, 0) == PAL_LOW;
  fwd_limit = palReadPad(GPIOA, 6) == PAL_HIGH;


  chThdCreateStatic(i2cslave_thread_wa, sizeof(i2cslave_thread_wa),
      NORMALPRIO, i2cslave_thread, NULL);
}

void app_i2cslave_configure(uint8_t controller_id) {
  mc_i2c_addr = controller_id;
}

/*
 * Thread for I2C Slave App
 *
 * Repeatedly updates the feedback struct at 100Hz and logs I2C errors to the console
 */
static THD_FUNCTION(i2cslave_thread, arg) 
{
  (void) arg;
  chRegSetThreadName("APP_I2CSLAVE");

  if (mc_i2c_addr == I2C_ADDR_GEN_CALL) {
    return;
  }

  i2cSlaveConfigure(&HW_I2C_DEV, &requestRx, &feedbackReply);
  i2cMatchAddress(&HW_I2C_DEV, mc_i2c_addr);

  while (true) {

    if (needFeeback) {
      updateFeedback();
      feedbackReply.size = sizeof(mc_feedback);
      if (i2cp_g != NULL) {
        i2cSlaveReply(i2cp_g, &feedbackReply);
      }
    }
    
    if (estop) {
      mc_interface_set_brake_current(0);
    } else if (rev_limit && mc_interface_get_duty_cycle_now() < 0) {
      mc_interface_brake_now();
    } else if (fwd_limit && mc_interface_get_duty_cycle_now() > 0) {
      mc_interface_brake_now();
    } else if (enable_homing) {
      homing_sequence();
    } else if (gotCommand) {
      setCommand();
      gotCommand = false;
    }

    if (gotI2cError)
    {
      gotI2cError = 0;
    }
    chThdSleepMilliseconds(10); // 100Hz
  } 
}
