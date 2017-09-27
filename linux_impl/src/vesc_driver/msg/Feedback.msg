float32 motor_current
int32 commanded_value
int32 measured_velocity
int32 measured_position
float32 supply_voltage
float32 supply_current
uint32 switch_flags

uint8 FAULT_CODE_NONE = 0
uint8 FAULT_CODE_OVER_VOLTAGE = 1
uint8 FAULT_CODE_UNDER_VOLTAGE = 2
uint8 FAULT_CODE_DRV8302 = 3
uint8 FAULT_CODE_ABS_OVER_CURRENT = 4
uint8 FAULT_CODE_OVER_TEMP_FET = 5
uint8 FAULT_CODE_OVER_TEMP_MOTOR = 6

uint8 status_code