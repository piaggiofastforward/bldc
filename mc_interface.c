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

/*
 * mc_interface.c
 *
 *  Created on: 10 okt 2015
 *      Author: benjamin
 */

#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "ledpwm.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "utils.h"
#include "ch.h"
#include "hal.h"
#include "encoder.h"
#include <math.h>

// Global variables
volatile uint16_t ADC_Value[HW_ADC_CHANNELS];
volatile int ADC_curr_norm_value[3];

// Private variables
static volatile mc_configuration m_conf;
static mc_fault_code m_fault_now;
static int m_ignore_iterations;
static volatile unsigned int m_cycles_running;
static volatile bool m_lock_enabled;
static volatile bool m_lock_override_once;
static volatile float m_motor_current_sum;
static volatile float m_input_current_sum;
static volatile float m_motor_current_iterations;
static volatile float m_input_current_iterations;
static volatile float m_amp_seconds;
static volatile float m_amp_seconds_charged;
static volatile float m_watt_seconds;
static volatile float m_watt_seconds_charged;
static volatile float m_position_set;
static int (*tacho_fun)(bool) = mcpwm_foc_get_tachometer_value;

// Sampling variables
#define ADC_SAMPLE_MAX_LEN		2000
static volatile int16_t m_curr0_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_curr1_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph1_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph2_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph3_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_vzero_samples[ADC_SAMPLE_MAX_LEN];
static volatile uint8_t m_status_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_curr_fir_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_f_sw_samples[ADC_SAMPLE_MAX_LEN];
static volatile int m_sample_len;
static volatile int m_sample_int;
static volatile int m_sample_ready;
static volatile int m_sample_now;
static volatile int m_sample_at_start;
static volatile int m_start_comm;
static volatile float m_last_adc_duration_sample;

// Private functions
static void update_override_limits(volatile mc_configuration *conf);

// Function pointers
static void(*pwn_done_func)(void) = 0;

// Threads
static THD_WORKING_AREA(timer_thread_wa, 1024);
static THD_FUNCTION(timer_thread, arg);

void mc_interface_init(mc_configuration *configuration) {
	m_conf = *configuration;
	m_fault_now = FAULT_CODE_NONE;
	m_ignore_iterations = 0;
	m_cycles_running = 0;
	m_lock_enabled = false;
	m_lock_override_once = false;
	m_motor_current_sum = 0.0;
	m_input_current_sum = 0.0;
	m_motor_current_iterations = 0.0;
	m_input_current_iterations = 0.0;
	m_amp_seconds = 0.0;
	m_amp_seconds_charged = 0.0;
	m_watt_seconds = 0.0;
	m_watt_seconds_charged = 0.0;
	m_position_set = 0.0;
	m_last_adc_duration_sample = 0.0;

	m_sample_len = 1000;
	m_sample_int = 1;
	m_sample_ready = 1;
	m_sample_now = 0;
	m_sample_at_start = 0;
	m_start_comm = 0;

	// Start threads
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

  mcpwm_foc_init(&m_conf);
}

const volatile mc_configuration* mc_interface_get_configuration(void) {
	return &m_conf;
}

void mc_interface_set_configuration(mc_configuration *configuration) {
  m_conf = *configuration;

	update_override_limits(&m_conf);

  mcpwm_foc_set_configuration(&m_conf);
}

void mc_interface_set_tacho_source(int (*tacho_src)(bool))
{
  tacho_fun = tacho_src;
}

bool mc_interface_dccal_done(void) {
  return mcpwm_foc_is_dccal_done();
}

/**
 * Set a function that should be called after each PWM cycle.
 *
 * @param p_func
 * The function to be called. 0 will not call any function.
 */
void mc_interface_set_pwm_callback(void (*p_func)(void)) {
	pwn_done_func = p_func;
}

/**
 * Lock the control by disabling all control commands.
 */
void mc_interface_lock(void) {
	m_lock_enabled = true;
}

/**
 * Unlock all control commands.
 */
void mc_interface_unlock(void) {
	m_lock_enabled = false;
}

/**
 * Allow just one motor control command in the locked state.
 */
void mc_interface_lock_override_once(void) {
	m_lock_override_once = true;
}

mc_fault_code mc_interface_get_fault(void) {
	return m_fault_now;
}

const char* mc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE"; break;
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE"; break;
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE"; break;
	case FAULT_CODE_DRV8302: return "FAULT_CODE_DRV8302"; break;
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT"; break;
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET"; break;
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR"; break;
	default: return "FAULT_UNKNOWN"; break;
	}
}

mc_state mc_interface_get_state(void) {
  return mcpwm_foc_get_state();
}

void mc_interface_set_duty(float dutyCycle) {
	if (mc_interface_try_input()) {
		return;
	}
  mcpwm_foc_set_duty(dutyCycle);
}

void mc_interface_set_duty_noramp(float dutyCycle) {
	if (mc_interface_try_input()) {
		return;
	}
  mcpwm_foc_set_duty_noramp(dutyCycle);
}

void mc_interface_set_pid_speed(float rpm) {
	if (mc_interface_try_input()) {
		return;
	}
  mcpwm_foc_set_pid_speed(rpm);
}

void mc_interface_set_pid_pos(int pos) {
	if (mc_interface_try_input()) {
		return;
	}

	m_position_set = pos;
  mcpwm_foc_set_pid_pos(pos);
}

void mc_interface_set_current(float current) {
	if (mc_interface_try_input()) {
		return;
	}
  mcpwm_foc_set_current(current);
}

void mc_interface_set_brake_current(float current) {
	if (mc_interface_try_input()) {
		return;
	}
  mcpwm_foc_set_brake_current(current);
}

void mc_interface_brake_now(void) {
	mc_interface_set_duty(0.0);
}

/**
 * Disconnect the motor and let it turn freely.
 */
void mc_interface_release_motor(void) {
	mc_interface_set_current(0.0);
}

/**
 * Stop the motor and use braking.
 */
float mc_interface_get_duty_cycle_set(void) {
  return mcpwm_foc_get_duty_cycle_set();
}

float mc_interface_get_duty_cycle_now(void) {
  return mcpwm_foc_get_duty_cycle_now();
}

float mc_interface_get_sampling_frequency_now(void) {
  return mcpwm_foc_get_switching_frequency_now() / 2.0;
}

float mc_interface_get_rpm(void) {
  return mcpwm_foc_get_rpm();
}

/**
 * Get the amount of amp hours drawn from the input source.
 *
 * @param reset
 * If true, the counter will be reset after this call.
 *
 * @return
 * The amount of amp hours drawn.
 */
float mc_interface_get_amp_hours(bool reset) {
	float val = m_amp_seconds / 3600;

	if (reset) {
		m_amp_seconds = 0.0;
	}

	return val;
}

/**
 * Get the amount of amp hours fed back into the input source.
 *
 * @param reset
 * If true, the counter will be reset after this call.
 *
 * @return
 * The amount of amp hours fed back.
 */
float mc_interface_get_amp_hours_charged(bool reset) {
	float val = m_amp_seconds_charged / 3600;

	if (reset) {
		m_amp_seconds_charged = 0.0;
	}

	return val;
}

/**
 * Get the amount of watt hours drawn from the input source.
 *
 * @param reset
 * If true, the counter will be reset after this call.
 *
 * @return
 * The amount of watt hours drawn.
 */
float mc_interface_get_watt_hours(bool reset) {
	float val = m_watt_seconds / 3600;

	if (reset) {
		m_amp_seconds = 0.0;
	}

	return val;
}

/**
 * Get the amount of watt hours fed back into the input source.
 *
 * @param reset
 * If true, the counter will be reset after this call.
 *
 * @return
 * The amount of watt hours fed back.
 */
float mc_interface_get_watt_hours_charged(bool reset) {
	float val = m_watt_seconds_charged / 3600;

	if (reset) {
		m_watt_seconds_charged = 0.0;
	}

	return val;
}

float mc_interface_get_tot_current(void) {
  return mcpwm_foc_get_tot_current();
}

float mc_interface_get_tot_current_filtered(void) {
  return mcpwm_foc_get_tot_current_filtered();
}

float mc_interface_get_tot_current_directional(void) {
  return mcpwm_foc_get_tot_current_directional();
}

float mc_interface_get_tot_current_directional_filtered(void) {
  return mcpwm_foc_get_tot_current_directional_filtered();
}

float mc_interface_get_tot_current_in(void) {
  return mcpwm_foc_get_tot_current_in();
}

float mc_interface_get_tot_current_in_filtered(void) {
  return mcpwm_foc_get_tot_current_in_filtered();
}

int mc_interface_get_tachometer_value(bool reset) {
  return tacho_fun(reset);
}

int mc_interface_get_tachometer_abs_value(bool reset) {
  return mcpwm_foc_get_tachometer_abs_value(reset);
}

float mc_interface_get_last_inj_adc_isr_duration(void) {
  return mcpwm_foc_get_last_inj_adc_isr_duration();
}

float mc_interface_read_reset_avg_motor_current(void) {
	float res = m_motor_current_sum / m_motor_current_iterations;
	m_motor_current_sum = 0;
	m_motor_current_iterations = 0;
	return res;
}

float mc_interface_read_reset_avg_input_current(void) {
	float res = m_input_current_sum / m_input_current_iterations;
	m_input_current_sum = 0;
	m_input_current_iterations = 0;
	return res;
}

int mc_interface_get_pid_pos_set(void) {
	return m_position_set;
}

int mc_interface_get_pid_pos_now(void) {
  return mcpwm_foc_get_pid_pos_now();
}

float mc_interface_get_last_sample_adc_isr_duration(void) {
	return m_last_adc_duration_sample;
}

// MC implementation functions
/**
 * A helper function that should be called before sending commands to control
 * the motor. If the state is detecting, the detection will be stopped.
 *
 * @return
 * The amount if milliseconds left until user commands are allowed again.
 *
 */
int mc_interface_try_input(void) {
	// TODO: Remove this later
	if (mc_interface_get_state() == MC_STATE_DETECTING) {
		mcpwm_foc_stop_pwm();
		m_ignore_iterations = MCPWM_FOC_DETECT_STOP_TIME;
	}

	int retval = m_ignore_iterations;

	if (!m_ignore_iterations && m_lock_enabled) {
		if (!m_lock_override_once) {
			retval = 1;
		} else {
			m_lock_override_once = false;
		}
	}

	return retval;
}

void mc_interface_fault_stop(mc_fault_code fault, bool isr) {
	if (mc_interface_dccal_done() && m_fault_now == FAULT_CODE_NONE) {
		// Sent to terminal fault logger so that all faults and their conditions
		// can be printed for debugging.
		isr ? chSysLockFromISR() : chSysLock();
		volatile int val_samp = TIM8->CCR1;
		volatile int current_samp = TIM1->CCR4;
		volatile int tim_top = TIM1->ARR;
		isr ? chSysUnlockFromISR() : chSysUnlock();

		fault_data fdata;
		fdata.fault = fault;
		fdata.current = mc_interface_get_tot_current();
		fdata.current_filtered = mc_interface_get_tot_current_filtered();
		fdata.voltage = GET_INPUT_VOLTAGE();
		fdata.duty = mc_interface_get_duty_cycle_now();
		fdata.rpm = mc_interface_get_rpm();
		fdata.tacho = mc_interface_get_tachometer_value(false);
		fdata.cycles_running = m_cycles_running;
		fdata.tim_val_samp = val_samp;
		fdata.tim_current_samp = current_samp;
		fdata.tim_top = tim_top;
		fdata.temperature = NTC_TEMP(ADC_IND_TEMP_MOS1);
    (void) fdata; //TODO log faults to i2c
	}

	m_ignore_iterations = m_conf.m_fault_stop_time_ms;
  mcpwm_foc_stop_pwm();

	m_fault_now = fault;
}

void mc_interface_mc_timer_isr(void) {
	ledpwm_update_pwm(); // LED PWM Driver update

	const float input_voltage = GET_INPUT_VOLTAGE();

	// Check for faults that should stop the motor
	static int wrong_voltage_iterations = 0;
	if (input_voltage < m_conf.l_min_vin ||
			input_voltage > m_conf.l_max_vin) {
		wrong_voltage_iterations++;

		if ((wrong_voltage_iterations >= 8)) {
			mc_interface_fault_stop(input_voltage < m_conf.l_min_vin ?
					FAULT_CODE_UNDER_VOLTAGE : FAULT_CODE_OVER_VOLTAGE, true);
		}
	} else {
		wrong_voltage_iterations = 0;
	}

	if (mc_interface_get_state() == MC_STATE_RUNNING) {
		m_cycles_running++;
	} else {
		m_cycles_running = 0;
	}

	if (pwn_done_func) {
		pwn_done_func();
	}

	const float current = mc_interface_get_tot_current_filtered();
	const float current_in = mc_interface_get_tot_current_in_filtered();
	m_motor_current_sum += current;
	m_input_current_sum += current_in;
	m_motor_current_iterations++;
	m_input_current_iterations++;

  float abs_current = mcpwm_foc_get_abs_motor_current();
  float abs_current_filtered = mcpwm_foc_get_abs_motor_current_filtered();

	// Current fault code
	if (m_conf.l_slow_abs_current) {
		if (fabsf(abs_current_filtered) > m_conf.l_abs_current_max) {
			mc_interface_fault_stop(FAULT_CODE_ABS_OVER_CURRENT, true);
		}
	} else {
		if (fabsf(abs_current) > m_conf.l_abs_current_max) {
			mc_interface_fault_stop(FAULT_CODE_ABS_OVER_CURRENT, true);
		}
	}

	// Watt and ah counters
	const float f_samp = mc_interface_get_sampling_frequency_now();
	if (fabsf(current) > 1.0) {
		// Some extra filtering
		static float curr_diff_sum = 0.0;
		static float curr_diff_samples = 0;

		curr_diff_sum += current_in / f_samp;
		curr_diff_samples += 1.0 / f_samp;

		if (curr_diff_samples >= 0.01) {
			if (curr_diff_sum > 0.0) {
				m_amp_seconds += curr_diff_sum;
				m_watt_seconds += curr_diff_sum * input_voltage;
			} else {
				m_amp_seconds_charged -= curr_diff_sum;
				m_watt_seconds_charged -= curr_diff_sum * input_voltage;
			}

			curr_diff_samples = 0.0;
			curr_diff_sum = 0.0;
		}
	}


}

void mc_interface_adc_inj_int_handler(void) {
  mcpwm_foc_adc_inj_int_handler();
}

/**
 * Update the override limits for a configuration based on MOSFET temperature etc.
 *
 * @param conf
 * The configaration to update.
 */
static void update_override_limits(volatile mc_configuration *conf) {
	const float temp = NTC_TEMP(ADC_IND_TEMP_MOS2);
	const float v_in = GET_INPUT_VOLTAGE();

	// Temperature
	if (temp < conf->l_temp_fet_start) {
		conf->lo_current_min = conf->l_current_min;
		conf->lo_current_max = conf->l_current_max;
	} else if (temp > conf->l_temp_fet_end) {
		conf->lo_current_min = 0.0;
		conf->lo_current_max = 0.0;
		mc_interface_fault_stop(FAULT_CODE_OVER_TEMP_FET, false);
	} else {
		float maxc = fabsf(conf->l_current_max);
		if (fabsf(conf->l_current_min) > maxc) {
			maxc = fabsf(conf->l_current_min);
		}

		maxc = utils_map(temp, conf->l_temp_fet_start, conf->l_temp_fet_end, maxc, 0.0);

		if (fabsf(conf->l_current_max) > maxc) {
			conf->lo_current_max = SIGN(conf->l_current_max) * maxc;
		}

		if (fabsf(conf->l_current_min) > maxc) {
			conf->lo_current_min = SIGN(conf->l_current_min) * maxc;
		}
	}

	// Battery cutoff
	if (v_in > conf->l_battery_cut_start) {
		conf->lo_in_current_max = conf->l_in_current_max;
	} else if (v_in < conf->l_battery_cut_end) {
		conf->lo_in_current_max = 0.0;
	} else {
		conf->lo_in_current_max = utils_map(v_in, conf->l_battery_cut_start,
				conf->l_battery_cut_end, conf->l_in_current_max, 0.0);
	}

	conf->lo_in_current_min = conf->l_in_current_min;
}

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("mcif timer");

	for(;;) {
		// Check if the DRV8302 indicates any fault
		if (IS_DRV_FAULT()) {
			mc_interface_fault_stop(FAULT_CODE_DRV8302, true);
		}

		// Decrease fault iterations
		if (m_ignore_iterations > 0) {
			m_ignore_iterations--;
		} else {
			if (!IS_DRV_FAULT()) {
				m_fault_now = FAULT_CODE_NONE;
			}
		}

		update_override_limits(&m_conf);

		chThdSleepMilliseconds(1);
	}
}

