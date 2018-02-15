/**
 *  Designed to break up a motor config XML into key/value pairs and send them to the VESC
 *  one by one. Each XML line will be translated into a CONFIG_WRITE command.
 */
#ifndef VESC_WRITE_CONFIG_H_
#define VESC_WRITE_CONFIG_H_
#include "inttypes.h"
#include <map>
#include <set>
#include "vesc_driver/control_msgs.h"

namespace vesc_config
{

/**
 *  The function passed to readAndSendXml should be a function with this signature which
 *  sends data over the wire.
 */
typedef void (*xmlSendFunc)(uint8_t *, unsigned int);

/**
 *  Signature for a function which commands the VESC to start the FOC hall detection process,
 *  then fills in the provided data array with the result.
 *
 *  A negative return value signals a timeout on the response to the request.
 */
typedef int (*focDetectFunc)(unsigned int, uint8_t*);

/**
 *  Read the contents of the XML located at "filename" and send it to the VESC.
 *  
 *  Return -1 if the file does not exist.
 *  Do NOT command the VESC to commit these changes. That should be handled by vesc_usb.cpp
 */
int readAndSendXml(const char* filename, xmlSendFunc sendFunc);

constexpr const int CONFIG_RESPONSE_TIMEOUT = 5;


/**
 *  Command the VESC to run its FOC hall detection routine. If it succeeds, update the XML configuration
 *  at the input file. If it doesnt succeed, dont modify.
 *
 *  NOTES:
 *    - the routine will NOT run if the file doesnt exist
 *    - this is designed so that it has no ROS dependencies and can be run from an executable
 *      without a ROS Master running.
 *
 */
int runHallFocDetection(const char* filename, focDetectFunc detectFunc);

constexpr const int FOC_DETECT_RESPONSE_TIMEOUT = 20;

/**
 *  Relate a key in the XML document to an enum value for easy VESC config commands.
 */
static std::set<std::string> ignored_xml_tags = {
  "motor_weight",
  "motor_brand",
  "motor_model",
  "motor_poles",
  "motor_quality_bearings",
  "motor_quality_magnets",
  "motor_quality_construction",
  "motor_sensor_type",
  "motor_loss_torque",
  "motor_description",
  "motor_quality_description",
};

static std::map<std::string, enum mc_config_param> xml_name_to_enum = 
{
  {"motor_type",                          MOTOR_TYPE},
  {"l_current_max",                       L_CURRENT_MAX},
  {"m_sensor_port_mode",                  M_SENSOR_PORT_MODE},
  {"foc_observer_gain_slow",              OBSERVER_GAIN_SLOW},
  {"foc_pll_kp",                          FOC_PLL_KP},
  {"sl_cycle_int_rpm_br",                 SL_CYCLE_INT_RPM},
  {"m_drv8301_oc_adj",                    M_DRV8301_OC_ADJ},
  {"l_in_current_max",                    L_IN_CURRENT_MAX},
  {"foc_sl_openloop_hyst",                OPENLOOP_HYST},
  {"foc_encoder_offset",                  ENCODER_OFFSET},
  {"l_temp_fet_end",                      L_TEMP_FET_END},
  {"foc_encoder_inverted",                ENCODER_INVERTED},
  {"foc_sl_openloop_time",                OPENLOOP_TIME},
  {"sl_min_erpm",                         SL_MIN_ERPM},
  {"l_battery_cut_end",                   BATTERY_CUT_END},
  {"foc_observer_gain",                   FOC_OBSERVER_GAIN},
  {"s_pid_allow_braking",                 PID_ALLOW_BRAKING},
  {"l_watt_max",                          WATT_MAX},
  {"sl_cycle_int_limit",                  SL_CYCLE_INT_LIMIT},
  {"foc_current_ki",                      FOC_CURRENT_KI},
  {"l_max_erpm_fbrake_cc",                MAX_EPRM_FBRAKE_CC},
  {"l_min_duty",                          MIN_DUTY},
  {"foc_current_kp",                      FOC_CURRENT_KP},
  {"m_duty_ramp_step",                    DUTY_RAMP_STEP},
  {"foc_sample_v0_v7",                    FOC_SAMPLE_V0_V7},
  {"l_abs_current_max",                   ABS_CURRENT_MAX},
  {"foc_temp_comp",                       FOC_TEMP_COMP},
  {"s_pid_min_erpm",                      S_PID_MIN_ERPM},
  {"m_ntc_motor_beta",                    M_NTC_MOTOR_BETA},
  {"l_max_duty",                          MAX_DUTY},
  {"foc_motor_flux_linkage",              MOTOR_FLUX_LINKAGE},
  {"l_max_erpm_fbrake",                   MAX_ERPM_FBRAKE},
  {"hall_sl_erpm",                        HALL_SL_ERPM},
  {"foc_sl_d_current_factor",             FOC_SL_D_CURRENT_FACTOR},
  {"cc_min_current",                      CC_MIN_CURRENT},
  {"cc_ramp_step_max",                    CC_RAMP_STEP_MAX},
  {"sensor_mode",                         SENSOR_MODE},
  {"foc_sample_high_current",             FOC_SAMPLE_HIGH_CURRENT},
  {"l_erpm_start",                        L_ERPM_START},
  {"foc_dt_us",                           FOC_DT_US},
  {"cc_gain",                             CC_GAIN},
  {"m_encoder_counts",                    M_ENCODER_COUNTS},
  {"l_max_vin",                           L_MAX_VIN},
  {"hall_table_0",                        HALL_TABLE},
  {"m_bldc_f_sw_min",                     M_BLDC_F_SW_MIN},
  {"hall_table_1",                        HALL_TABLE},
  {"hall_table_2",                        HALL_TABLE},
  {"hall_table_3",                        HALL_TABLE},
  {"hall_table_4",                        HALL_TABLE},
  {"foc_hall_table_0",                    HALL_TABLE_FOC},
  {"hall_table_5",                        HALL_TABLE},
  {"foc_hall_table_1",                    HALL_TABLE_FOC},
  {"hall_table_6",                        HALL_TABLE},
  {"pwm_mode",                            PWM_MODE},
  {"hall_table_7",                        HALL_TABLE},
  {"foc_hall_table_2",                    HALL_TABLE_FOC},
  {"p_pid_kd",                            P_PID_KD},
  {"l_temp_motor_end",                    L_TEMP_MOTOR_END},
  {"foc_hall_table_3",                    HALL_TABLE_FOC},
  {"foc_hall_table_4",                    HALL_TABLE_FOC},
  {"foc_sat_comp",                        FOC_SAT_COMP},
  {"foc_hall_table_5",                    HALL_TABLE_FOC},
  {"foc_hall_table_6",                    HALL_TABLE_FOC},
  {"p_pid_ang_div",                       P_PID_ANG_DIV},
  {"sl_max_fullbreak_current_dir_change", MAX_FULLBREAK_CURRENT_DIR_CHANGE},
  {"l_battery_cut_start",                 L_BATTERY_CUT_START},
  {"foc_hall_table_7",                    HALL_TABLE_FOC},
  {"p_pid_ki",                            P_PID_KI},
  {"m_invert_direction",                  M_INVERT_DIRECTION},
  {"foc_sl_d_current_duty",               FOC_SL_D_CURRENT_DUTY},
  {"cc_startup_boost_duty",               CC_STARTUP_BOOST_DUTY},
  {"foc_temp_comp_base_temp",             FOC_TEMP_COMP_BASE_TEMP},
  {"p_pid_kp",                            S_PID_KP},
  {"l_min_erpm",                          L_MIN_ERPM},
  {"foc_sl_erpm",                         FOC_SL_ERPM},
  {"m_fault_stop_time_ms",                M_FAULT_STOP_TIME_MS},
  {"m_bldc_f_sw_max",                     M_BLDC_F_SW_MAX},
  {"s_pid_kd",                            S_PID_KD},
  {"l_temp_fet_start",                    L_TEMP_FET_START},
  {"l_max_erpm",                          L_MAX_ERPM},
  {"sl_phase_advance_at_br",              SL_PHASE_ADVANCE_AT_BR},
  {"foc_motor_l",                         FOC_MOTOR_L},
  {"s_pid_ki",                            S_PID_KI,},
  {"m_drv8301_oc_mode",                   M_DRV8301_OC_MODE},
  {"l_min_vin",                           L_MIN_VIN},
  {"foc_motor_r",                         FOC_MOTOR_R},
  {"foc_duty_dowmramp_ki",                FOC_DUTY_DOWNRAMP_KI},
  {"comm_mode",                           COMM_MODE},
  {"s_pid_kp",                            S_PID_KP},
  {"foc_duty_dowmramp_kp",                FOC_DUTY_DOWNRAMP_KP},
  {"l_current_min",                       L_CURRENT_MIN},
  {"foc_encoder_ratio",                   FOC_ENCODER_RATIO},
  {"l_in_current_min",                    L_IN_CURRENT_MIN},
  {"m_current_backoff_gain",              M_CURRENT_BACKOFF_GAIN},
  {"sl_bemf_coupling_k",                  SL_BEMF_COUPLING_K},
  {"sl_min_erpm_cycle_int_limit",         SL_MIN_ERPM_CYCLE_INT_LIMIT},
  {"l_temp_motor_start",                  L_TEMP_MOTOR_START},
  {"foc_openloop_rpm",                    OPENLOOP_RPM},
  {"l_slow_abs_current",                  L_SLOW_ABS_CURRENT},
  {"foc_sensor_mode",                     FOC_SENSOR_MODE},
  {"m_dc_f_sw",                           M_DC_F_SW},
  {"foc_f_sw",                            FOC_F_SW},
  {"foc_pll_ki",                          FOC_PLL_KI},
  {"l_watt_min",                          L_WATT_MIN},
};

} // vesc_config

#endif
