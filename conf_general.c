/*
 * conf_general.c
 *
 *  Created on: 14 sep 2014
 *      Author: benjamin
 */

#include "conf_general.h"
#include "ch.h"
#include "eeprom.h"
#include "mc_interface.h"
#include "hw.h"
#include "utils.h"
#include "stm32f4xx_conf.h"
#include "timeout.h"

#include <string.h>
#include <math.h>

// User defined default motor configuration file
#ifdef MCCONF_DEFAULT_USER
#include MCCONF_DEFAULT_USER
#endif

// User defined default app configuration file
#ifdef APPCONF_DEFAULT_USER
#include APPCONF_DEFAULT_USER
#endif

// Default configuration parameters that can be overridden
#include "mcconf_default.h"
#include "appconf_default.h"

// EEPROM settings
#define EEPROM_BASE_MCCONF		1000
#define EEPROM_BASE_APPCONF		2000

// Global variables
uint16_t VirtAddVarTab[NB_OF_VAR];

// Private variables
mc_configuration mcconf, mcconf_old;

void conf_general_init(void) {
	// First, make sure that all relevant virtual addresses are assigned for page swapping.
	memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

	int ind = 0;
	for (unsigned int i = 0;i < (sizeof(app_configuration) / 2);i++) {
		VirtAddVarTab[ind++] = EEPROM_BASE_MCCONF + i;
	}

	for (unsigned int i = 0;i < (sizeof(app_configuration) / 2);i++) {
		VirtAddVarTab[ind++] = EEPROM_BASE_APPCONF + i;
	}

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	EE_Init();
}

/**
 * Load the compiled default app_configuration.
 *
 * @param conf
 * A pointer to store the default configuration to.
 */
void conf_general_get_default_app_configuration(app_configuration *conf) {
	memset(conf, 0, sizeof(app_configuration));
	conf->controller_id = APPCONF_CONTROLLER_ID;
	conf->timeout_msec = APPCONF_TIMEOUT_MSEC;
	conf->timeout_brake_current = APPCONF_TIMEOUT_BRAKE_CURRENT;
	conf->send_can_status = APPCONF_SEND_CAN_STATUS;
	conf->send_can_status_rate_hz = APPCONF_SEND_CAN_STATUS_RATE_HZ;

	conf->app_to_use = APPCONF_APP_TO_USE;

	conf->app_ppm_conf.ctrl_type = APPCONF_PPM_CTRL_TYPE;
	conf->app_ppm_conf.pid_max_erpm = APPCONF_PPM_PID_MAX_ERPM;
	conf->app_ppm_conf.hyst = APPCONF_PPM_HYST;
	conf->app_ppm_conf.pulse_start = APPCONF_PPM_PULSE_START;
	conf->app_ppm_conf.pulse_end = APPCONF_PPM_PULSE_END;
	conf->app_ppm_conf.median_filter = APPCONF_PPM_MEDIAN_FILTER;
	conf->app_ppm_conf.safe_start = APPCONF_PPM_SAFE_START;
	conf->app_ppm_conf.rpm_lim_start = APPCONF_PPM_RPM_LIM_START;
	conf->app_ppm_conf.rpm_lim_end = APPCONF_PPM_RPM_LIM_END;
	conf->app_ppm_conf.multi_esc = APPCONF_PPM_MULTI_ESC;
	conf->app_ppm_conf.tc = APPCONF_PPM_TC;
	conf->app_ppm_conf.tc_max_diff = APPCONF_PPM_TC_MAX_DIFF;

	conf->app_adc_conf.ctrl_type = APPCONF_ADC_CTRL_TYPE;
	conf->app_adc_conf.hyst = APPCONF_ADC_HYST;
	conf->app_adc_conf.voltage_start = APPCONF_ADC_VOLTAGE_START;
	conf->app_adc_conf.voltage_end = APPCONF_ADC_VOLTAGE_END;
	conf->app_adc_conf.use_filter = APPCONF_ADC_USE_FILTER;
	conf->app_adc_conf.safe_start = APPCONF_ADC_SAFE_START;
	conf->app_adc_conf.cc_button_inverted = APPCONF_ADC_CC_BUTTON_INVERTED;
	conf->app_adc_conf.rev_button_inverted = APPCONF_ADC_REV_BUTTON_INVERTED;
	conf->app_adc_conf.voltage_inverted = APPCONF_ADC_VOLTAGE_INVERTED;
	conf->app_adc_conf.rpm_lim_start = APPCONF_ADC_RPM_LIM_START;
	conf->app_adc_conf.rpm_lim_end = APPCONF_ADC_RPM_LIM_END;
	conf->app_adc_conf.multi_esc = APPCONF_ADC_MULTI_ESC;
	conf->app_adc_conf.tc = APPCONF_ADC_TC;
	conf->app_adc_conf.tc_max_diff = APPCONF_ADC_TC_MAX_DIFF;
	conf->app_adc_conf.update_rate_hz = APPCONF_ADC_UPDATE_RATE_HZ;

	conf->app_uart_baudrate = APPCONF_UART_BAUDRATE;

	conf->app_chuk_conf.ctrl_type = APPCONF_CHUK_CTRL_TYPE;
	conf->app_chuk_conf.hyst = APPCONF_CHUK_HYST;
	conf->app_chuk_conf.rpm_lim_start = APPCONF_CHUK_RPM_LIM_START;
	conf->app_chuk_conf.rpm_lim_end = APPCONF_CHUK_RPM_LIM_END;
	conf->app_chuk_conf.ramp_time_pos = APPCONF_CHUK_RAMP_TIME_POS;
	conf->app_chuk_conf.ramp_time_neg = APPCONF_CHUK_RAMP_TIME_NEG;
	conf->app_chuk_conf.stick_erpm_per_s_in_cc = APPCONF_STICK_ERPM_PER_S_IN_CC;
	conf->app_chuk_conf.multi_esc = APPCONF_CHUK_MULTI_ESC;
	conf->app_chuk_conf.tc = APPCONF_CHUK_TC;
	conf->app_chuk_conf.tc_max_diff = APPCONF_CHUK_TC_MAX_DIFF;

	conf->app_nrf_conf.speed = APPCONF_NRF_SPEED;
	conf->app_nrf_conf.power = APPCONF_NRF_POWER;
	conf->app_nrf_conf.crc_type = APPCONF_NRF_CRC;
	conf->app_nrf_conf.retry_delay = APPCONF_NRF_RETR_DELAY;
	conf->app_nrf_conf.retries = APPCONF_NRF_RETRIES;
	conf->app_nrf_conf.channel = APPCONF_NRF_CHANNEL;
	conf->app_nrf_conf.address[0] = APPCONF_NRF_ADDR_B0;
	conf->app_nrf_conf.address[1] = APPCONF_NRF_ADDR_B1;
	conf->app_nrf_conf.address[2] = APPCONF_NRF_ADDR_B2;
	conf->app_nrf_conf.send_crc_ack = APPCONF_NRF_SEND_CRC_ACK;
}

/**
 * Load the compiled default mc_configuration.
 *
 * @param conf
 * A pointer to store the default configuration to.
 */
void conf_general_get_default_mc_configuration(mc_configuration *conf) {
	memset(conf, 0, sizeof(mc_configuration));

	conf->l_current_max = MCCONF_L_CURRENT_MAX;
	conf->l_current_min = MCCONF_L_CURRENT_MIN;
	conf->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
	conf->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
	conf->l_abs_current_max = MCCONF_L_MAX_ABS_CURRENT;
	conf->l_min_erpm = MCCONF_L_RPM_MIN;
	conf->l_max_erpm = MCCONF_L_RPM_MAX;
	conf->l_max_erpm_fbrake = MCCONF_L_CURR_MAX_RPM_FBRAKE;
	conf->l_max_erpm_fbrake_cc = MCCONF_L_CURR_MAX_RPM_FBRAKE_CC;
	conf->l_min_vin = MCCONF_L_MIN_VOLTAGE;
	conf->l_max_vin = MCCONF_L_MAX_VOLTAGE;
	conf->l_battery_cut_start = MCCONF_L_BATTERY_CUT_START;
	conf->l_battery_cut_end = MCCONF_L_BATTERY_CUT_END;
	conf->l_slow_abs_current = MCCONF_L_SLOW_ABS_OVERCURRENT;
	conf->l_rpm_lim_neg_torque = MCCONF_L_RPM_LIMIT_NEG_TORQUE;
	conf->l_temp_fet_start = MCCONF_L_LIM_TEMP_FET_START;
	conf->l_temp_fet_end = MCCONF_L_LIM_TEMP_FET_END;
	conf->l_temp_motor_start = MCCONF_L_LIM_TEMP_MOTOR_START;
	conf->l_temp_motor_end = MCCONF_L_LIM_TEMP_MOTOR_END;
	conf->l_min_duty = MCCONF_L_MIN_DUTY;
	conf->l_max_duty = MCCONF_L_MAX_DUTY;

	conf->lo_current_max = conf->l_current_max;
	conf->lo_current_min = conf->l_current_min;
	conf->lo_in_current_max = conf->l_in_current_max;
	conf->lo_in_current_min = conf->l_in_current_min;

	conf->sl_min_erpm = MCCONF_SL_MIN_RPM;
	conf->sl_max_fullbreak_current_dir_change = MCCONF_SL_MAX_FB_CURR_DIR_CHANGE;
	conf->sl_min_erpm_cycle_int_limit = MCCONF_SL_MIN_ERPM_CYCLE_INT_LIMIT;
	conf->sl_cycle_int_limit = MCCONF_SL_CYCLE_INT_LIMIT;
	conf->sl_phase_advance_at_br = MCCONF_SL_PHASE_ADVANCE_AT_BR;
	conf->sl_cycle_int_rpm_br = MCCONF_SL_CYCLE_INT_BR;
	conf->sl_bemf_coupling_k = MCCONF_SL_BEMF_COUPLING_K;

	conf->hall_table[0] = MCCONF_HALL_TAB_0;
	conf->hall_table[1] = MCCONF_HALL_TAB_1;
	conf->hall_table[2] = MCCONF_HALL_TAB_2;
	conf->hall_table[3] = MCCONF_HALL_TAB_3;
	conf->hall_table[4] = MCCONF_HALL_TAB_4;
	conf->hall_table[5] = MCCONF_HALL_TAB_5;
	conf->hall_table[6] = MCCONF_HALL_TAB_6;
	conf->hall_table[7] = MCCONF_HALL_TAB_7;
	conf->hall_sl_erpm = MCCONF_HALL_ERPM;

	conf->foc_current_kp = MCCONF_FOC_CURRENT_KP;
	conf->foc_current_ki = MCCONF_FOC_CURRENT_KI;
	conf->foc_f_sw = MCCONF_FOC_F_SW;
	conf->foc_dt_us = MCCONF_FOC_DT_US;
	conf->foc_encoder_inverted = MCCONF_FOC_ENCODER_INVERTED;
	conf->foc_encoder_offset = MCCONF_FOC_ENCODER_OFFSET;
	conf->foc_encoder_ratio = MCCONF_FOC_ENCODER_RATIO;
	conf->foc_pll_kp = MCCONF_FOC_PLL_KP;
	conf->foc_pll_ki = MCCONF_FOC_PLL_KI;
	conf->foc_motor_l = MCCONF_FOC_MOTOR_L;
	conf->foc_motor_r = MCCONF_FOC_MOTOR_R;
	conf->foc_motor_flux_linkage = MCCONF_FOC_MOTOR_FLUX_LINKAGE;
	conf->foc_observer_gain = MCCONF_FOC_OBSERVER_GAIN;
	conf->foc_duty_dowmramp_kp = MCCONF_FOC_DUTY_DOWNRAMP_KP;
	conf->foc_duty_dowmramp_ki = MCCONF_FOC_DUTY_DOWNRAMP_KI;
	conf->foc_openloop_rpm = MCCONF_FOC_OPENLOOP_RPM;
	conf->foc_sl_openloop_hyst = MCCONF_FOC_SL_OPENLOOP_HYST;
	conf->foc_sl_openloop_time = MCCONF_FOC_SL_OPENLOOP_TIME;
	conf->foc_sl_d_current_duty = MCCONF_FOC_SL_D_CURRENT_DUTY;
	conf->foc_sl_d_current_factor = MCCONF_FOC_SL_D_CURRENT_FACTOR;
	conf->foc_hall_table[0] = MCCONF_FOC_HALL_TAB_0;
	conf->foc_hall_table[1] = MCCONF_FOC_HALL_TAB_1;
	conf->foc_hall_table[2] = MCCONF_FOC_HALL_TAB_2;
	conf->foc_hall_table[3] = MCCONF_FOC_HALL_TAB_3;
	conf->foc_hall_table[4] = MCCONF_FOC_HALL_TAB_4;
	conf->foc_hall_table[5] = MCCONF_FOC_HALL_TAB_5;
	conf->foc_hall_table[6] = MCCONF_FOC_HALL_TAB_6;
	conf->foc_hall_table[7] = MCCONF_FOC_HALL_TAB_7;
	conf->foc_sl_erpm = MCCONF_FOC_SL_ERPM;

	conf->s_pid_kp = MCCONF_S_PID_KP;
	conf->s_pid_ki = MCCONF_S_PID_KI;
	conf->s_pid_kd = MCCONF_S_PID_KD;
	conf->s_pid_min_erpm = MCCONF_S_PID_MIN_RPM;

	conf->p_pid_kp = MCCONF_P_PID_KP;
	conf->p_pid_ki = MCCONF_P_PID_KI;
	conf->p_pid_kd = MCCONF_P_PID_KD;
	conf->p_pid_ang_div = MCCONF_P_PID_ANG_DIV;

	conf->cc_startup_boost_duty = MCCONF_CC_STARTUP_BOOST_DUTY;
	conf->cc_min_current = MCCONF_CC_MIN_CURRENT;
	conf->cc_gain = MCCONF_CC_GAIN;
	conf->cc_ramp_step_max = MCCONF_CC_RAMP_STEP;

	conf->m_fault_stop_time_ms = MCCONF_M_FAULT_STOP_TIME;
	conf->m_duty_ramp_step = MCCONF_M_RAMP_STEP;
	conf->m_duty_ramp_step_rpm_lim = MCCONF_M_RAMP_STEP_RPM_LIM;
	conf->m_current_backoff_gain = MCCONF_M_CURRENT_BACKOFF_GAIN;
	conf->m_encoder_counts = MCCONF_M_ENCODER_COUNTS;
	conf->m_sensor_port_mode = MCCONF_M_SENSOR_PORT_MODE;
}

/**
 * Read app_configuration from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a app_configuration struct to write the read configuration to.
 */
void conf_general_read_app_configuration(app_configuration *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	for (unsigned int i = 0;i < (sizeof(app_configuration) / 2);i++) {
		if (EE_ReadVariable(EEPROM_BASE_APPCONF + i, &var) == 0) {
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		} else {
			is_ok = false;
			break;
		}
	}

	// Set the default configuration
	if (!is_ok) {
		conf_general_get_default_app_configuration(conf);
	}
}

/**
 * Write app_configuration to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_app_configuration(app_configuration *conf) {
	mc_interface_unlock();
	mc_interface_release_motor();

	utils_sys_lock_cnt();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);

	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	for (unsigned int i = 0;i < (sizeof(app_configuration) / 2);i++) {
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;

		if (EE_WriteVariable(EEPROM_BASE_APPCONF + i, var) != FLASH_COMPLETE) {
			is_ok = false;
			break;
		}
	}

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	utils_sys_unlock_cnt();

	return is_ok;
}

/**
 * Read mc_configuration from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a mc_configuration struct to write the read configuration to.
 */
void conf_general_read_mc_configuration(mc_configuration *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	for (unsigned int i = 0;i < (sizeof(mc_configuration) / 2);i++) {
		if (EE_ReadVariable(EEPROM_BASE_MCCONF + i, &var) == 0) {
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		} else {
			is_ok = false;
			break;
		}
	}

	if (!is_ok) {
		conf_general_get_default_mc_configuration(conf);
	}
}

/**
 * Write mc_configuration to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_mc_configuration(mc_configuration *conf) {
	mc_interface_unlock();
	mc_interface_release_motor();

	utils_sys_lock_cnt();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);

	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	for (unsigned int i = 0;i < (sizeof(mc_configuration) / 2);i++) {
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;

		if (EE_WriteVariable(EEPROM_BASE_MCCONF + i, var) != FLASH_COMPLETE) {
			is_ok = false;
			break;
		}
	}

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	utils_sys_unlock_cnt();

	return is_ok;
}

