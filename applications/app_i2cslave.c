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
#include <string.h>
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
void configReplyDone(I2CDriver *i2cp);
void setCommand(void);
void noteI2cError(uint32_t flags);
void updateFeedback(void);
void app_i2cslave_init(void);
int  descale_position(float pos);
void reset_homing(void);
void homing_sequence(void);
volatile float *getParamPtr(enum mc_config_param param);
inline static float getParameter(enum mc_config_param param);

// Thread for VESC application
static THD_FUNCTION(i2cslave_thread, arg);
static THD_WORKING_AREA(i2cslave_thread_wa, 2048);


static volatile mc_request command_now = {{0}, {0}, 0};
static mc_request_union request;

static mc_feedback_union fb;
static mc_status_union status;
static volatile bool needFeeback = false;
static volatile bool gotCommand = false;
static I2CDriver *i2cp_g = NULL;
static i2caddr_t mc_i2c_addr = 0x00; // default I2C Address, by design will crash the app
static volatile mc_configuration mcconf;

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

mc_config_union config;
void configReplyDone(I2CDriver *i2cp) {
  (void) i2cp;
  i2cSlaveReplyI(i2cp, &feedbackReply);
}

// Response after VESC_STATUS is written
static const I2CSlaveMsg configReply = {  // this is in RAM so size may be updated
  sizeof(config.value_bytes),  // Zero so clock will stretch until response is ready 
  config.value_bytes,                // Status buffer
  NULL,                        // message already full
  configReplyDone,             // Reset to default feedback reply
  catchError                   // Error hook
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

void changeResponse(I2CDriver *i2cp, 
                    enum mc_request_type response, 
                    enum mc_config_param param) {
  switch (response) {
    case FEEDBACK_READ:
      i2cSlaveReplyI(i2cp, &feedbackReply);
      break;
    case STATUS_READ:
      i2cSlaveReplyI(i2cp, &statusReply);
      break;
    case CONFIG_READ:
      if (request.request.param == HALL_TABLE) {
        memcpy(config.value.hall_table, 
               (void *) mc_interface_get_configuration()->foc_hall_table, 
               HALL_TABLE_SIZE);
      } else {
        config.value.value = getParameter(param);
      }
      i2cSlaveReplyI(i2cp, &configReply);
      break;
    default:
      break;
  }
}

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
      return 0;
  }
}

inline static float getParameter(enum mc_config_param param)
{
  return *getParamPtr(param);
}

inline static void setParameter(enum mc_config_param param, float value)
{
  *getParamPtr(param) = value;
}

inline static void setHall(hall_table_t hall_table)
{
  memcpy((void *) mcconf.foc_hall_table, hall_table, HALL_TABLE_SIZE);
}

bool updateConfig = false;

/*
 * Callback for received messages
 * Handles either setting a command or changing the response for the next read
 * Called in interrupt context
 */
void requestProcessor(I2CDriver *i2cp)
{ 
  switch (request.request.type) {
    case FEEDBACK_READ:
    case STATUS_READ:
    case CONFIG_READ:
      changeResponse(i2cp, request.request.type, request.request.param);
      break;
    case CONTROL_WRITE:
      command_now = request.request;
      gotCommand = true;
      timeout_reset();
      break;
    case CONFIG_WRITE:
      if (request.request.param == HALL_TABLE) {
        setHall(request.request.value_hall);
      } else {
        setParameter(request.request.param, request.request.value_f); 
      }
      updateConfig = true;
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
  switch (command_now.control) {
    case SPEED:
      fb.feedback.commanded_velocity = command_now.value_i;
      mc_interface_set_pid_speed(command_now.value_i);
      break;
    case CURRENT:
      fb.feedback.commanded_velocity = command_now.value_f;
      mc_interface_set_current(command_now.value_f);
      break;
    case DUTY:
      fb.feedback.commanded_velocity = command_now.value_f;
      mc_interface_set_duty(command_now.value_f);
      break;
    case POSITION:
      fb.feedback.commanded_velocity = command_now.value_i;
      mc_interface_set_pid_pos(command_now.value_i);
      break;
    case SCALE_POS:
      fb.feedback.commanded_velocity = command_now.value_f;
      mc_interface_set_pid_pos(descale_position(command_now.value_f));
      break;
    case HOMING: 
      fb.feedback.commanded_velocity = -1;
      homing_sequence();
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
static bool rev_limit = false;
static bool fwd_limit = false;
/*
 * Updates the feedback struct with current data
 */ void updateFeedback()
{
  fb.feedback.motor_current = mc_interface_get_tot_current() * 1000;
  fb.feedback.measured_velocity = mc_interface_get_rpm();
  fb.feedback.measured_position = mc_interface_get_tachometer_value(false);
  fb.feedback.supply_voltage  = GET_INPUT_VOLTAGE() * 1000;
  fb.feedback.supply_current = mc_interface_get_tot_current_in() * 1000;
  fb.feedback.switch_flags = ST2MS(chVTGetSystemTimeX());
  /* fb.feedback.switch_flags = (estop << 2) | (rev_limit << 1) | (fwd_limit); */
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
      mc_interface_set_current(8);
    } 
  } else if (min_tacho == INT_MAX) {
    if (!rev_limit) {
      mc_interface_set_current(-8);
    }
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

/* void toggle_rev_limit(EXTDriver *extp, expchannel_t channel)  */
/* { */
/*   (void) extp; */
/*   (void) channel; */
/*   rev_limit = palReadPad(GPIOA, 6) == PAL_HIGH; */
/*   if (rev_limit) { */
/*     if (mc_interface_get_duty_cycle_now() < 0) { */
/*       mc_interface_brake_now(); */
/*     } */
/*     min_tacho = mc_interface_get_tachometer_value(false); */
/*   } */
/* } */

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
    /* {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, toggle_rev_limit}, */
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
 * Initialization for the i2c slave control
 * Starts the I2C bus and configures it to respond 
 */
void app_i2cslave_init() 
{
  hw_start_i2c();

  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_PULLDOWN);
  /* palSetPadMode(GPIOA, 6, PAL_MODE_INPUT); */
  palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_PULLUP);
  extStart(&EXTD1, &extcfg);

  estop = palReadPad(GPIOA, 5) == PAL_LOW;
  /* rev_limit = palReadPad(GPIOA, 6) == PAL_LOW; */
  rev_limit = false;
  fwd_limit = palReadPad(GPIOC, 0) == PAL_LOW;
  mcconf = *mc_interface_get_configuration();

  chThdCreateStatic(i2cslave_thread_wa, sizeof(i2cslave_thread_wa),
      NORMALPRIO, i2cslave_thread, NULL);
}

void app_i2cslave_configure(uint8_t controller_id) {
  mc_i2c_addr = controller_id;
}


bool shouldMove(void) {
  bool isForward = true;
  switch (command_now.control) {
    case POSITION:
      isForward = command_now.value_i - mc_interface_get_tachometer_value(false) > 0;
      break;
    case SCALE_POS:
      isForward = descale_position(command_now.value_f / 1000.0f) > 0 - 
        mc_interface_get_tachometer_value(false);
      break;
    case SPEED:
      isForward = command_now.value_i > 0;
      break;
    case HOMING:
      if (max_tacho == INT_MIN) {
        isForward = true;
      } else if (min_tacho == INT_MAX) {
        isForward = false;
      }
    case CURRENT:
    case DUTY:
      isForward = command_now.value_f > 0;
      break;
    
    /* default: */
  }

  // Return false if the limit switch for the current direction is triggered
  return !(isForward ? fwd_limit : rev_limit);
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

  i2cSlaveConfigure(&HW_I2C_DEV, &requestRx, &feedbackReply);
  i2cMatchAddress(&HW_I2C_DEV, mc_i2c_addr);

  while (true) {
    if (estop) {
      mc_interface_set_brake_current(0);
    } else if (!shouldMove()) {
      mc_interface_brake_now();
    } else if (gotCommand) {
      setCommand();
      gotCommand = false;
    }

    if (updateConfig) {
      mc_interface_set_configuration((mc_configuration *) &mcconf);
      conf_general_store_mc_configuration((mc_configuration *) &mcconf);
      updateConfig = false;
    }

    if (needFeeback) {
      updateFeedback();
      feedbackReply.size = sizeof(mc_feedback);
      if (i2cp_g != NULL) {
        i2cSlaveReply(i2cp_g, &feedbackReply);
      }
    }

    if (gotI2cError)
    {
      gotI2cError = 0;
    }
    chThdSleepMilliseconds(1); // 100Hz
  } 
}
