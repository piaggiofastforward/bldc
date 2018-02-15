float32 motor_current
int32 commanded_value
int32 measured_velocity
int32 measured_position
float32 supply_voltage
float32 supply_current
float32 last_pid_current_output


# use logical AND on the switch flags eg. switch_flag = 5 means that both
# ESTOP and REV_LIMIT are set...
#
#  if (feedback_message & ESTOP)
#    // estop is set
#  if (feedback_message & REV_LIMIT)
#    // rev_limit is set
#
uint32 REV_LIMIT = 1
uint32 FWD_LIMIT = 2
uint32 ESTOP     = 4
uint32 switch_flags
