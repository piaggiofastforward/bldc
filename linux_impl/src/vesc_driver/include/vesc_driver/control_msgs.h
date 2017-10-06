#ifndef __I2C_MSGS_H__
#define __I2C_MSGS_H__
#include <stdint.h>
#include <stdbool.h>

#define ENUM_SIZE : uint8_t

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
enum mc_packet_type ENUM_SIZE {
  FEEDBACK_DATA = 1,
  CONFIG_READ,
  STATUS_DATA,
  CONTROL_WRITE,
  CONFIG_WRITE
};

// An enum to differentiate control modes
enum mc_control_mode ENUM_SIZE {
  SPEED = 1, // use value_i
  CURRENT, // use value_f
  POSITION, // use value_i
  DUTY, // use value_f
  HOMING, // no value set
  SCALE_POS // use value_f
};

// An enum to differentiate config parameters
// use value_f in request
// use value in config_value response
enum mc_config_param ENUM_SIZE {
  FOC_KP = 1, 
  FOC_KI,
  MOTOR_L,
  MOTOR_R,
  MOTOR_FLUX,
  OBSERVER_GAIN,
  MAX_CURRENT,
  MIN_CURRENT,
  POS_PID_KP,
  POS_PID_KI,
  POS_PID_KD,
  SPEED_PID_KP,
  SPEED_PID_KI,
  SPEED_PID_KD,
  SPEED_PID_MIN_RPM,
  HALL_TABLE // use hall_table
};

// The hall table is an array of eight values determined by bldc-tool
#define HALL_TABLE_SIZE 8
typedef uint8_t hall_table_t[HALL_TABLE_SIZE];

// A struct that represents the received requests
typedef struct {
  union {
    int32_t value_i; 
    float value_f;
    hall_table_t value_hall; // Only used for CONFIG_WRITE HALL_TABLE
  };
  union {
    enum mc_control_mode control;
    enum mc_config_param param;
  };
  enum mc_packet_type type;
} mc_request;

// A union to simplify sending a request over i2c
typedef union {
  mc_request request;
  uint8_t request_bytes[sizeof(mc_request)];
} mc_request_union;

/*************************
 * VESC CONFIG DATATYPES *
 *************************/
// the struct that represents the current configuration for a parameter
typedef union {
  hall_table_t hall_table;
  float value;
} mc_config_value;

// a union to simplify transferring config values over i2c
typedef union {
  mc_config_value value;
  uint8_t value_bytes[sizeof(mc_config_value)];
} mc_config_union;
 
/***************************
 * VESC FEEDBACK DATATYPES *
 ***************************/
// The struct that represents a feedback message
typedef struct {
  float motor_current;
  int32_t commanded_value;
  int32_t measured_velocity;
  int32_t measured_position;
  float supply_voltage;
  float supply_current;
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

#endif
