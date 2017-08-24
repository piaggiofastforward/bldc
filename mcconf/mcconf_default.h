/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MCCONF_DEFAULT_H_
#define MCCONF_DEFAULT_H_

// Default settings
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_PWM_MODE
#define MCCONF_PWM_MODE					PWM_MODE_SYNCHRONOUS // Default PWM mode
#endif
#ifndef MCCONF_SENSOR_MODE
#define MCCONF_SENSOR_MODE				SENSOR_MODE_HALL // Sensor mode
#endif
//
// Limits
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX			60.0	// Current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN			-60.0	// Current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			60.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-20.0	// Input current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		130.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			8.0		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			57.0	// Maximum input voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_START
#define MCCONF_L_BATTERY_CUT_START		10.0	// Start limiting the positive current at this voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_END
#define MCCONF_L_BATTERY_CUT_END		8.0		// Limit the positive current completely at this voltage
#endif
#ifndef MCCONF_L_RPM_MAX
#define MCCONF_L_RPM_MAX				100000.0	// The motor speed limit (Upper)
#endif
#ifndef MCCONF_L_RPM_MIN
#define MCCONF_L_RPM_MIN				-100000.0	// The motor speed limit (Lower)
#endif
#ifndef MCCONF_L_SLOW_ABS_OVERCURRENT
#define MCCONF_L_SLOW_ABS_OVERCURRENT	true	// Use the filtered (and hence slower) current for the overcurrent fault detection
#endif
#ifndef MCCONF_L_MIN_DUTY
#define MCCONF_L_MIN_DUTY				0.005	// Minimum duty cycle
#endif
#ifndef MCCONF_L_MAX_DUTY
#define MCCONF_L_MAX_DUTY				0.95	// Maximum duty cycle
#endif
#ifndef MCCONF_L_RPM_LIMIT_NEG_TORQUE
#define MCCONF_L_RPM_LIMIT_NEG_TORQUE	true	// Use negative torque to limit the RPM
#endif
#ifndef MCCONF_L_CURR_MAX_RPM_FBRAKE
#define MCCONF_L_CURR_MAX_RPM_FBRAKE	300		// Maximum electrical RPM to use full brake at
#endif
#ifndef MCCONF_L_CURR_MAX_RPM_FBRAKE_CC
#define MCCONF_L_CURR_MAX_RPM_FBRAKE_CC	1500	// Maximum electrical RPM to use full brake at with current control
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_START
#define MCCONF_L_LIM_TEMP_FET_START		80.0	// MOSFET temperature where current limiting should begin
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_END
#define MCCONF_L_LIM_TEMP_FET_END		100.0	// MOSFET temperature where everything should be shut off
#endif
#ifndef MCCONF_L_LIM_TEMP_MOTOR_START
#define MCCONF_L_LIM_TEMP_MOTOR_START	80.0	// MOTOR temperature where current limiting should begin
#endif
#ifndef MCCONF_L_LIM_TEMP_MOTOR_END
#define MCCONF_L_LIM_TEMP_MOTOR_END		100.0	// MOTOR temperature where everything should be shut off
#endif

// Speed PID parameters
#ifndef MCCONF_S_PID_KP
#define MCCONF_S_PID_KP					0.0	// Proportional gain
#endif
#ifndef MCCONF_S_PID_KI
#define MCCONF_S_PID_KI					0.0	// Integral gain
#endif
#ifndef MCCONF_S_PID_KD
#define MCCONF_S_PID_KD					0.000		// Derivative gain
#endif
#ifndef MCCONF_S_PID_MIN_RPM
#define MCCONF_S_PID_MIN_RPM			900.0	// Minimum allowed RPM
#endif

// Position PID parameters
#ifndef MCCONF_P_PID_KP
#define MCCONF_P_PID_KP					0.03	// Proportional gain
#endif
#ifndef MCCONF_P_PID_KI
#define MCCONF_P_PID_KI					0.0000		// Integral gain
#endif
#ifndef MCCONF_P_PID_KD
#define MCCONF_P_PID_KD					0.0004	// Derivative gain
#endif

// FOC
#ifndef MCCONF_FOC_CURRENT_KP
#define MCCONF_FOC_CURRENT_KP			0.0121
#endif
#ifndef MCCONF_FOC_CURRENT_KI
#define MCCONF_FOC_CURRENT_KI			9.4
#endif
#ifndef MCCONF_FOC_F_SW
#define MCCONF_FOC_F_SW					20000.0
#endif
#ifndef MCCONF_FOC_DT_US
#define MCCONF_FOC_DT_US				0.08 // Microseconds for dead time compensation
#endif
#ifndef MCCONF_FOC_ENCODER_INVERTED
#define MCCONF_FOC_ENCODER_INVERTED		false
#endif
#ifndef MCCONF_FOC_ENCODER_OFFSET
#define MCCONF_FOC_ENCODER_OFFSET		180.0
#endif
#ifndef MCCONF_FOC_ENCODER_RATIO
#define MCCONF_FOC_ENCODER_RATIO		7.0
#endif
#ifndef MCCONF_FOC_PLL_KP
#define MCCONF_FOC_PLL_KP				2000.0
#endif
#ifndef MCCONF_FOC_PLL_KI
#define MCCONF_FOC_PLL_KI				20000.0
#endif
#ifndef MCCONF_FOC_MOTOR_L
#define MCCONF_FOC_MOTOR_L				0.00012066
#endif
#ifndef MCCONF_FOC_MOTOR_R
#define MCCONF_FOC_MOTOR_R				0.09397
#endif
#ifndef MCCONF_FOC_MOTOR_FLUX_LINKAGE
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE	.001169
#endif
#ifndef MCCONF_FOC_OBSERVER_GAIN
#define MCCONF_FOC_OBSERVER_GAIN		8.26e6 // Can be something like 600 / L
#endif
#ifndef MCCONF_FOC_DUTY_DOWNRAMP_KP
#define MCCONF_FOC_DUTY_DOWNRAMP_KP		10.0	// PI controller for duty control when decreasing the duty
#endif
#ifndef MCCONF_FOC_DUTY_DOWNRAMP_KI
#define MCCONF_FOC_DUTY_DOWNRAMP_KI		200.0	// PI controller for duty control when decreasing the duty
#endif
#ifndef MCCONF_FOC_OPENLOOP_RPM
#define MCCONF_FOC_OPENLOOP_RPM			1200.0	// Openloop RPM (sensorless low speed or when finding index pulse)
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_HYST
#define MCCONF_FOC_SL_OPENLOOP_HYST		0.5		// Time below min RPM to activate openloop (s)
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_TIME
#define MCCONF_FOC_SL_OPENLOOP_TIME		0.5		// Time to remain in openloop (s)
#endif
#ifndef MCCONF_FOC_SL_D_CURRENT_DUTY
#define MCCONF_FOC_SL_D_CURRENT_DUTY	0.0		// Inject d-axis current below this duty cycle in sensorless more
#endif
#ifndef MCCONF_FOC_SL_D_CURRENT_FACTOR
#define MCCONF_FOC_SL_D_CURRENT_FACTOR	0.0		// Maximum q-axis current factor
#endif
#ifndef MCCONF_FOC_HALL_TAB_0
#define MCCONF_FOC_HALL_TAB_0			255
#endif
#ifndef MCCONF_FOC_HALL_TAB_1
#define MCCONF_FOC_HALL_TAB_1			134
#endif
#ifndef MCCONF_FOC_HALL_TAB_2
#define MCCONF_FOC_HALL_TAB_2			197
#endif
#ifndef MCCONF_FOC_HALL_TAB_3
#define MCCONF_FOC_HALL_TAB_3			166
#endif
#ifndef MCCONF_FOC_HALL_TAB_4
#define MCCONF_FOC_HALL_TAB_4			68
#endif
#ifndef MCCONF_FOC_HALL_TAB_5
#define MCCONF_FOC_HALL_TAB_5			98
#endif
#ifndef MCCONF_FOC_HALL_TAB_6
#define MCCONF_FOC_HALL_TAB_6			34
#endif
#ifndef MCCONF_FOC_HALL_TAB_7
#define MCCONF_FOC_HALL_TAB_7			255
#endif
#ifndef MCCONF_FOC_SL_ERPM
#define MCCONF_FOC_SL_ERPM				2500.0	// ERPM above which only the observer is used
#endif

// Misc
#ifndef MCCONF_M_FAULT_STOP_TIME
#define MCCONF_M_FAULT_STOP_TIME		3000	// Ignore commands for this duration in msec when faults occur
#endif
#ifndef MCCONF_M_ENCODER_COUNTS
#define MCCONF_M_ENCODER_COUNTS			8192	// The number of encoder counts
#endif
#ifndef MCCONF_M_SENSOR_PORT_MODE
#define MCCONF_M_SENSOR_PORT_MODE		SENSOR_PORT_MODE_HALL // The mode of the hall_encoder port
#endif

#endif /* MCCONF_DEFAULT_H_ */
