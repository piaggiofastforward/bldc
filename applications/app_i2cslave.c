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
#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "mcpwm.h" // Motor control functions
#include "hw.h" // HW Pin Mapping
#include "timeout.h" // Reset MC Timeout
#include "i2cslave.h"
#include "i2c_lld.h"
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

// Thread for VESC application
static THD_FUNCTION(i2cslave_thread, arg);
static THD_WORKING_AREA(i2cslave_thread_wa, 2048);

// Feedback struct to store feedback info between requests
typedef struct {
  /* uint32_t motor_current; */
  /* uint32_t commanded_velocity; */
  /* uint32_t measured_velocity; */
  /* uint32_t measured_position; */
  /* uint32_t supply_voltage; */
  /* uint32_t supply_current; */
  uint32_t hall;
  uint32_t comm_step;
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

static volatile mc_command command_now = {0, 0};

static mc_feedback_union fb;
static volatile bool needFeeback = false;
static volatile bool gotCommand = false;
static I2CDriver *i2cp_g = NULL;
static i2caddr_t mc_i2c_addr = 0x00; // default I2C Address, by design will crash the app

// Buffers for I2C comms
static uint8_t statusBody[16];

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
  statusBody,                // Status buffer
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
  int error;
  switch (command_now.type) {
    case VESC_CONTROL_SPEED:
      mc_interface_set_pid_speed(command_now.value);
      /* fb.feedback.commanded_velocity = command_now.value; */
      break;
    case VESC_CONTROL_ROTOR:
      mc_interface_set_pid_pos(command_now.value);
      /* fb.feedback.commanded_velocity = 0; */
      break;
    case VESC_CONTROL_CURRENT:
      mc_interface_set_current(command_now.value);
      /* fb.feedback.commanded_velocity = 0; */
      break;
    case VESC_CONTROL_DUTY:
      mc_interface_set_duty(command_now.value);
      /* fb.feedback.commanded_velocity = 0; */
      break;
    case VESC_CONTROL_POSITION:
      error = command_now.value - mc_interface_get_tachometer_value(false);
      mc_interface_set_duty(.004 * error);
      /* fb.feedback.commanded_velocity = 0; */
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
  const uint8_t fault_code = mc_interface_get_fault();
  const float temp = NTC_TEMP(ADC_IND_TEMP_MOS2);
  int size = snprintf((char *) statusBody, sizeof(statusBody), "%d:%.3f", fault_code, (double) temp);
  statusReply.size = size;
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

/*
 * Updates the feedback struct with current data
 */
void updateFeedback()
{
  /* fb.feedback.motor_current = mc_interface_get_tot_current() * 1000; */
  /* fb.feedback.measured_velocity = mc_interface_get_rpm(); */
  /* fb.feedback.measured_position = mc_interface_get_tachometer_value(false); */
  /* fb.feedback.supply_voltage  = GET_INPUT_VOLTAGE() * 1000; */
  /* fb.feedback.supply_current = mc_interface_get_tot_current_in() * 1000; */
  fb.feedback.hall = read_hall();
  fb.feedback.comm_step = mcpwm_get_comm_step();
}

/*
 * Initialization for the i2c slave control
 * Starts the I2C bus and configures it to respond 
 */
void app_i2cslave_init() 
{
  hw_start_i2c();

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
    
    if (gotCommand) {
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
