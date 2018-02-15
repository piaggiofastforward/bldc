#ifndef __I2C_MSGS_H__
#define __I2C_MSGS_H__
#include <stdint.h>
#include <stdbool.h>


/***************************
 * VESC REQUEST DATATYPES  *
 ***************************/

/**
 * An enum to differentiate the type of request.
 * 
 * - Status and feedback are sent from VESC -> Linux
 * - CONTROL_WRITE, and CONFIG_WRITE are sent from Linux -> VESC  
 * - CONFIG_READ is sent in both directions (Linux -> VESC query as well as VESC -> Linux response)
 */
enum mc_packet_type {
  FEEDBACK_DATA = 1,
  CONFIG_READ,
  STATUS_DATA,
  CONTROL_WRITE,
  CONFIG_WRITE,
  CONFIG_WRITE_HALL,        // only for HALL_TABLE and HALL_TABLE_FOC
  CONFIG_WRITE_CURRENT_PID, // only for writing current control PID values
  COMMIT_MC_CONFIG,         // use after many CONFIG_WRITE*s in order to actually effect the changes
  REQUEST_DETECT_HALL_FOC,  // use to perform FOC hall table calibration routine
  RESPONSE_DETECT_HALL_FOC, // use for response containing foc hall table data
};

/**
 *  8 value bytes + 1 bytes (success/fail)
 */
#define RESPONSE_DETECT_HALL_FOC_SIZE 9

// An enum to differentiate control modes
enum mc_control_mode {
  SPEED = 1, // use value_i
  CURRENT, // use value_i, units are in milliamps
  POSITION, // use value_i
  DUTY, // use value_f
  HOMING, // no value set
  SCALE_POS // use value_f
};

// An enum to differentiate config parameters
// use value_f in request
// use value in config_value response
enum mc_config_param {
  L_CURRENT_MAX = 1,
  M_SENSOR_PORT_MODE,
  OBSERVER_GAIN_SLOW,
  FOC_PLL_KP,
  SL_CYCLE_INT_RPM,
  M_DRV8301_OC_ADJ,
  L_IN_CURRENT_MAX,
  OPENLOOP_HYST,
  ENCODER_OFFSET,
  MOTOR_QUALITY_BEARINGS,
  L_TEMP_FET_END,
  MOTOR_QUALITY_MAGNETS,
  ENCODER_INVERTED,
  OPENLOOP_TIME,
  SL_MIN_ERPM,
  BATTERY_CUT_END,
  FOC_OBSERVER_GAIN,
  PID_ALLOW_BRAKING,
  WATT_MAX,
  SL_CYCLE_INT_LIMIT,
  FOC_CURRENT_KI,
  MAX_EPRM_FBRAKE_CC,
  MIN_DUTY,
  FOC_CURRENT_KP,
  DUTY_RAMP_STEP,
  FOC_SAMPLE_V0_V7,
  ABS_CURRENT_MAX,
  FOC_TEMP_COMP,
  S_PID_MIN_ERPM,
  M_NTC_MOTOR_BETA,
  MAX_DUTY,
  MOTOR_FLUX_LINKAGE,
  MAX_ERPM_FBRAKE,
  HALL_SL_ERPM,
  FOC_SL_D_CURRENT_FACTOR,
  CC_MIN_CURRENT,
  CC_RAMP_STEP_MAX,
  SENSOR_MODE,
  FOC_SAMPLE_HIGH_CURRENT,
  L_ERPM_START,
  FOC_DT_US,
  CC_GAIN,
  M_ENCODER_COUNTS,
  L_MAX_VIN,
  HALL_TABLE_0,
  M_BLDC_F_SW_MIN,
  MOTOR_WEIGHT,
  PWM_MODE,
  P_PID_KD,
  L_TEMP_MOTOR_END,
  FOC_SAT_COMP,
  P_PID_ANG_DIV,
  MAX_FULLBREAK_CURRENT_DIR_CHANGE,
  L_BATTERY_CUT_START,
  P_PID_KI,
  M_INVERT_DIRECTION,
  FOC_SL_D_CURRENT_DUTY,
  MOTOR_TYPE,
  CC_STARTUP_BOOST_DUTY,
  FOC_TEMP_COMP_BASE_TEMP,
  P_PID_KP,
  L_MIN_ERPM,
  FOC_SL_ERPM,
  M_FAULT_STOP_TIME_MS,
  MOTOR_BRAND,
  M_BLDC_F_SW_MAX,
  S_PID_KD,
  L_TEMP_FET_START,
  L_MAX_ERPM,
  SL_PHASE_ADVANCE_AT_BR,
  FOC_MOTOR_L,
  S_PID_KI,
  M_DRV8301_OC_MODE,
  L_MIN_VIN,
  FOC_MOTOR_R,
  FOC_DUTY_DOWNRAMP_KI,
  COMM_MODE,
  MOTOR_QUALITY_CONSTRUCTION,
  S_PID_KP,
  FOC_DUTY_DOWNRAMP_KP,
  L_CURRENT_MIN,
  FOC_ENCODER_RATIO,
  L_IN_CURRENT_MIN,
  M_CURRENT_BACKOFF_GAIN,
  SL_BEMF_COUPLING_K,
  MOTOR_SENSOR_TYPE,
  MOTOR_MODEL,
  SL_MIN_ERPM_CYCLE_INT_LIMIT,
  L_TEMP_MOTOR_START,
  OPENLOOP_RPM,
  L_SLOW_ABS_CURRENT,
  FOC_SENSOR_MODE,
  MOTOR_LOSS_TORQUE,
  M_DC_F_SW,
  FOC_F_SW,
  MOTOR_POLES,
  FOC_PLL_KI,
  L_WATT_MIN,

  // These values will each be arrays of 8 bytes
  HALL_TABLE,
  HALL_TABLE_FOC,
};


/**
 *  This struct should be used to give BOTH speed and current commands. The __attribute__ is used
 *  to ensure that the struct below is the exact size of the parameters within, with no extra
 *  padding.
 */
typedef struct {
  int32_t target_cmd_i;
  enum mc_control_mode control_mode;
} mc_cmd;


typedef union {
  mc_cmd cmd;
  uint8_t cmd_bytes[sizeof(mc_cmd)];
} mc_cmd_union;

typedef struct {
  float kp;
  float ki;
  float kd;
} mc_config_current_pid;

typedef union {
  mc_config_current_pid config;
  uint8_t config_bytes[sizeof(mc_config_current_pid)];
} mc_config_current_pid_union;

typedef struct {
  union {
    float    value_f;
    int32_t  value_i;
    uint8_t  value_byte;
    uint32_t value_ui; // value_unsigned_int
  };
  enum mc_config_param param;
} mc_config;

typedef union {
  mc_config config;
  uint8_t config_bytes[sizeof(mc_config)];
} mc_config_union;

/**
 *  The hall table is an array of eight values determined by bldc-tool.
 *  HALL_TABLE     is in range [-1, 6]
 *  HALL_TABLE_FOC is in range [0, 255]
 */
#define HALL_TABLE_SIZE 8
typedef int8_t  hall_table_t[HALL_TABLE_SIZE];
typedef uint8_t hall_table_foc_t[HALL_TABLE_SIZE];

/**
 *  Hall tables (for both FOC and BLDC) consist of 8 values. If we included these in the mc_config_union,
 *  all values would be forced to be the size of the largest transaction - hall table config writes.
 *  The enum and union below were introduced to avoid that situation.
 */
typedef struct {
  union {
    hall_table_t hall_values;
    hall_table_foc_t hall_foc_values;
  };
  enum mc_config_param param;
} mc_config_hall;

typedef union {
  mc_config_hall config;
  uint8_t config_bytes[sizeof(mc_config_hall)];
} mc_config_hall_union;
 
/***************************
 * VESC FEEDBACK DATATYPES *
 ***************************/
// The struct that represents a feedback message
typedef struct {
  float motor_current;
  int32_t measured_velocity;
  uint32_t measured_position;
  float supply_voltage;
  float supply_current;
  float last_pid_current_output;
  uint32_t switch_flags;
} mc_feedback;

// a union to simplify sending feedback over i2c
typedef union {
  mc_feedback feedback;
  uint8_t feedback_bytes[sizeof(mc_feedback)];
} mc_feedback_union;

/*************************
 * VESC STATUS DATATYPES *
 *************************/
// The struct that represents a status message
typedef struct {
  uint32_t fault_code;
  uint32_t temp;
  bool limits_set;
} mc_status;

// a union to simplify transferring statuses over i2c
typedef union {
  mc_status status;
  uint8_t status_bytes[sizeof(mc_status)];
} mc_status_union;



typedef void (*packetSendFunc)(uint8_t*, unsigned int);

// Send the command with the specified sending function
void sendCommand(packetSendFunc sendFunc, mc_cmd cmd);

// extract the command from a data buffer.
// Return -1 if this is not possible, and 0 otherwise
int extractCommand(const uint8_t* data, const unsigned int size, mc_cmd *cmd);

int extractStatusData(const uint8_t* data, const unsigned int size, mc_status_union *status);
void sendStatusData(packetSendFunc sendFunc, const mc_status_union status);
int extractFeedbackData(const uint8_t* data, const unsigned int size, mc_feedback_union *fb);
void sendFeedbackData(packetSendFunc sendFunc, const mc_feedback_union fb);

int extractCurrentPIDData(const uint8_t* data, const unsigned int size, mc_config_current_pid_union *config);
void sendCurrentPIDData(packetSendFunc sendFunc, const mc_config_current_pid_union config);

// these macros can be used if you are ABSOLUTELY SURE about what structure the packet contains.
// use at ur own risk :)))) F is for "fast" I suppose
#define extractStatusDataF(status_union, buf) (memcpy(status_union.status_bytes, buf + 1, sizeof(mc_status)))
#define extractFeedbackDataF(feedback_union, buf) (memcpy(feedback_union.feedback_bytes, buf + 1, sizeof(mc_feedback)))
#define extractCurrentPIDDataF(cfg_union, buf) (memcpy(cfg_union.config_bytes, buf + 1, sizeof(mc_config_current_pid)))

#endif
