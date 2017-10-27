#include "uart_mc_config.h"

  /*
   * Returns a pointer to the parameter in the local configuration
   */
static void setFloatParam(enum mc_config_param param, float val, mc_configuration *mcconfig)
{
  switch (param) 
  {
    case L_CURRENT_MAX:
      mcconfig->l_current_max = val;
      return;
    case OBSERVER_GAIN_SLOW:
      mcconfig->foc_observer_gain_slow = val;
      return;
    case FOC_PLL_KP:
      mcconfig->foc_pll_kp = val;
      return;
    case SL_CYCLE_INT_RPM:
      mcconfig->sl_cycle_int_rpm_br = val;
      return;
    case L_IN_CURRENT_MAX:
      mcconfig->l_in_current_max = val;
      return;
    case OPENLOOP_HYST:
      mcconfig->foc_sl_openloop_hyst = val;
      return;
    case ENCODER_OFFSET:
      mcconfig->foc_encoder_offset = val;
      return;
    case L_TEMP_FET_END:
      mcconfig->l_temp_fet_end = val;
      return;
    case OPENLOOP_TIME:
      mcconfig->foc_sl_openloop_time = val;
      return;
    case SL_MIN_ERPM:
      mcconfig->sl_min_erpm = val;
      return;
    case BATTERY_CUT_END:
      mcconfig->l_battery_cut_end = val;
      return;
    case FOC_OBSERVER_GAIN:
      mcconfig->foc_observer_gain = val;
      return;
    case WATT_MAX:
      mcconfig->l_watt_max = val;
      return;
    case SL_CYCLE_INT_LIMIT:
      mcconfig->sl_cycle_int_limit = val;
      return;
    case FOC_CURRENT_KI:
      mcconfig->foc_current_ki = val;
      return;
    case MAX_EPRM_FBRAKE_CC:
      mcconfig->l_max_erpm_fbrake_cc = val;
      return;
    case MIN_DUTY:
      mcconfig->l_min_duty = val;
      return;
    case FOC_CURRENT_KP:
      mcconfig->foc_current_kp = val;
      return;
    case DUTY_RAMP_STEP:
      mcconfig->m_duty_ramp_step = val;
      return;
    case ABS_CURRENT_MAX:
      mcconfig->l_abs_current_max = val;
      return;
    case S_PID_MIN_ERPM:
      mcconfig->s_pid_min_erpm = val;
      return;
    case M_NTC_MOTOR_BETA:
      mcconfig->m_ntc_motor_beta = val;
      return;
    case MAX_DUTY:
      mcconfig->l_max_duty = val;
      return;
    case MOTOR_FLUX_LINKAGE:
      mcconfig->foc_motor_flux_linkage = val;
      return;
    case MAX_ERPM_FBRAKE:
      mcconfig->l_max_erpm_fbrake = val;
      return;
    case HALL_SL_ERPM:
      mcconfig->hall_sl_erpm = val;
      return;
    case FOC_SL_D_CURRENT_FACTOR:
      mcconfig->foc_sl_d_current_factor = val;
      return;
    case CC_MIN_CURRENT:
      mcconfig->cc_min_current = val;
      return;
    case CC_RAMP_STEP_MAX:
      mcconfig->cc_ramp_step_max = val;
      return;
    case L_ERPM_START:
      mcconfig->l_erpm_start = val;
      return;
    case FOC_DT_US:
      mcconfig->foc_dt_us = val;
      return;
    case CC_GAIN:
      mcconfig->cc_gain = val;
      return;
    case L_MAX_VIN:
      mcconfig->l_max_vin = val;
      return;
    case M_BLDC_F_SW_MIN:
      mcconfig->m_bldc_f_sw_min = val;
      return;
    case P_PID_KD:
      mcconfig->p_pid_kd = val;
      return;
    case L_TEMP_MOTOR_END:
      mcconfig->l_temp_motor_end = val;
      return;
    case FOC_SAT_COMP:
      mcconfig->foc_sat_comp = val;
      return;
    case P_PID_ANG_DIV:
      mcconfig->p_pid_ang_div = val;
      return;
    case MAX_FULLBREAK_CURRENT_DIR_CHANGE:
      mcconfig->sl_max_fullbreak_current_dir_change = val;
      return;
    case L_BATTERY_CUT_START:
      mcconfig->l_battery_cut_start = val;
      return;
    case P_PID_KI:
      mcconfig->p_pid_ki = val;
      return;
    case FOC_SL_D_CURRENT_DUTY:
      mcconfig->foc_sl_d_current_duty = val;
      return;
    case CC_STARTUP_BOOST_DUTY:
      mcconfig->cc_startup_boost_duty = val;
      return;
    case FOC_TEMP_COMP_BASE_TEMP:
      mcconfig->foc_temp_comp_base_temp = val;
      return;
    case P_PID_KP:
      mcconfig->p_pid_kp = val;
      return;
    case L_MIN_ERPM:
      mcconfig->l_min_erpm = val;
      return;
    case FOC_SL_ERPM:
      mcconfig->foc_sl_erpm = val;
      return;
    case M_BLDC_F_SW_MAX:
      mcconfig->m_bldc_f_sw_max = val;
      return;
    case S_PID_KD:
      mcconfig->s_pid_kd = val;
      return;
    case L_TEMP_FET_START:
      mcconfig->l_temp_fet_start = val;
      return;
    case L_MAX_ERPM:
      mcconfig->l_max_erpm = val;
      return;
    case SL_PHASE_ADVANCE_AT_BR:
      mcconfig->sl_phase_advance_at_br = val;
      return;
    case FOC_MOTOR_L:
      mcconfig->foc_motor_l = val;
      return;
    case S_PID_KI:
      mcconfig->s_pid_ki = val;
      return;
    case L_MIN_VIN:
      mcconfig->l_min_vin = val;
      return;
    case FOC_MOTOR_R:
      mcconfig->foc_motor_r = val;
      return;
    case FOC_DUTY_DOWNRAMP_KI:
      mcconfig->foc_duty_dowmramp_ki = val;
      return;
    case S_PID_KP:
      mcconfig->s_pid_kp = val;
      return;
    case FOC_DUTY_DOWNRAMP_KP:
      mcconfig->foc_duty_dowmramp_kp = val;
      return;
    case L_CURRENT_MIN:
      mcconfig->l_current_min = val;
      return;
    case FOC_ENCODER_RATIO:
      mcconfig->foc_encoder_ratio = val;
      return;
    case L_IN_CURRENT_MIN:
      mcconfig->l_in_current_min = val;
      return;
    case M_CURRENT_BACKOFF_GAIN:
      mcconfig->m_current_backoff_gain = val;
      return;
    case SL_BEMF_COUPLING_K:
      mcconfig->sl_bemf_coupling_k = val;
      return;
    case SL_MIN_ERPM_CYCLE_INT_LIMIT:
      mcconfig->sl_min_erpm_cycle_int_limit = val;
      return;
    case L_TEMP_MOTOR_START:
      mcconfig->l_temp_motor_start = val;
      return;
    case OPENLOOP_RPM:
      mcconfig->foc_openloop_rpm = val;
      return;
    case M_DC_F_SW:
      mcconfig->m_dc_f_sw = val;
      return;
    case FOC_F_SW:
      mcconfig->foc_f_sw = val;
      return;
    case FOC_PLL_KI: 
      mcconfig->foc_pll_ki = val;
      return;
    case L_WATT_MIN:
      mcconfig->l_watt_min = val;
      return;
    default:
      return;
  }
}

/*
 * Sets the parameter to the provided value in the local configuration
 *
 * Does not update the configuration externally
 */
void setParameter(mc_config config, mc_configuration *mcconfig)
{
  switch(config.param)
  {
    /**
     *  This case includes bools and uint8_t - anything that's a single byte
     */
    case PWM_MODE:
      mcconfig->pwm_mode = config.value_byte;
      break;
    case COMM_MODE:
      mcconfig->comm_mode = config.value_byte;
      break;
    case MOTOR_TYPE:
      mcconfig->motor_type = config.value_byte;
      break;
    case SENSOR_MODE:
      mcconfig->sensor_mode = config.value_byte;
      break;
    case L_SLOW_ABS_CURRENT:
      mcconfig->l_slow_abs_current = config.value_byte;
      break;
    case ENCODER_INVERTED:
      mcconfig->foc_encoder_inverted = config.value_byte;
      break;
    case FOC_SAMPLE_V0_V7:
      mcconfig->foc_sample_v0_v7 = config.value_byte;
      break;
    case FOC_SAMPLE_HIGH_CURRENT:
      mcconfig->foc_sample_high_current = config.value_byte;
      break;
    case FOC_TEMP_COMP:
      mcconfig->foc_temp_comp = config.value_byte;
      break;
    case FOC_SENSOR_MODE:
      mcconfig->foc_sensor_mode = config.value_byte;
      break;
    case PID_ALLOW_BRAKING:
      mcconfig->s_pid_allow_braking = config.value_byte;
      break;
    case M_SENSOR_PORT_MODE:
      mcconfig->m_sensor_port_mode = config.value_byte;
      break;
    case M_DRV8301_OC_MODE:
      mcconfig->m_drv8301_oc_mode = config.value_byte;
      break;
    case M_INVERT_DIRECTION:
      mcconfig->m_invert_direction = config.value_byte;
      break;

    /**
     *  32-bit signed fields
     */
    case M_FAULT_STOP_TIME_MS:
        mcconfig->m_fault_stop_time_ms = config.value_i;
        break;
      case M_DRV8301_OC_ADJ:
        mcconfig->m_drv8301_oc_adj = config.value_i;
        break;

    /**
     *  32-bit unsigned fields
     */
    case M_ENCODER_COUNTS:
      mcconfig->m_encoder_counts = config.value_ui;
      break;

    // default - the rest are floats
    default:
      setFloatParam(config.param, config.value_f, mcconfig);
      break;
    }

  }

