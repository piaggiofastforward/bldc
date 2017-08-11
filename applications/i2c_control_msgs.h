#ifndef __I2C_MSGS_H__
#define __I2C_MSGS_H__

enum mc_request_type {
  FEEDBACK_READ = 1,
  CONFIG_READ,
  STATUS_READ,
  CONTROL_WRITE,
  CONFIG_WRITE
};

enum mc_control_type {
  SPEED = 1,
  CURRENT,
  POSITION,
  DUTY,
  HOMING,
  SCALE_POS
};

#define HALL_TABLE_SIZE 8

typedef uint8_t hall_table_t[HALL_TABLE_SIZE];

enum mc_config_param {
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
  HALL_TABLE
};

typedef struct {
  union {
    int32_t value_i; 
    float value_f;
    hall_table_t value_hall;
  };
  union {
    enum mc_control_type control;
    enum mc_config_param param;
  };
  enum mc_request_type type;
} mc_request;

typedef union {
  mc_request request;
  uint8_t request_bytes[sizeof(mc_request)];
} mc_request_union;

typedef union {
  hall_table_t hall_table;
  float value;
} mc_config_value;

typedef union {
  mc_config_value value;
  uint8_t value_bytes[sizeof(mc_config_value)];
} mc_config_union;
 
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
  int32_t fault_code;
  int32_t temp;
} mc_status;

typedef union {
  mc_status status;
  uint8_t status_bytes[sizeof(mc_status)];
} mc_status_union;

#endif
