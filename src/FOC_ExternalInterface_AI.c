
#include "FOC_ExternalInterface.h"

#include <string.h>

#include "foc_api.h"
#include "foc_config.h"
#include "foc_core.h"
#include "foc_hal_if.h"
#include "foc_math.h"

#ifdef __ICCARM__
#define FOC_AI_DEBUG_ROOT __root
#else
#define FOC_AI_DEBUG_ROOT
#endif

FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_selected_motor_id = 0U;
extern volatile uint8_t g_foc_dyn_speed_start_on_max_fdb;
extern volatile float speed_ref;

#define FOC_DYN_SPEED_LOG_SIZE 512U
#define FOC_DETAIL_LOG_SIZE    512U

#define FOC_TEST_CASE_STOP              0U
#define FOC_TEST_CASE_FIXED_SPEED       1U
#define FOC_TEST_CASE_DYN_SPEED_CW      2U
#define FOC_TEST_CASE_BIDIR_SWITCH      3U
#define FOC_TEST_CASE_DYN_SPEED_CCW     4U

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_dyn_speed_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_dyn_speed_reverse = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_dyn_speed_start_on_max_ref = 1U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_start_cmd_rpm = 4500;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_dyn_speed_period_ms = 4000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_speed_min_rpm = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_speed_max_rpm = 4000U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_dyn_speed_reset_stats = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_last_ext_ref_rpm = 0;

FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_dyn_speed_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_speed_phase_u16 = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_ref_rpm = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_fdb_rpm = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_ctrl_fdb_rpm = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_err_rpm = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_speed_abs_err_rpm = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_speed_abs_err_avg_rpm = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_speed_max_abs_err_rpm = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_iq_ref_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_id_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_speed_iq_mA = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_speed_current_peak_mA = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_dyn_speed_sample_count = 0U;

FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_log_idx = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_dyn_log_stop = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_log_decim_ms = 50U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_dyn_log_t_ms[FOC_DYN_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_log_ref_rpm[FOC_DYN_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_log_fdb_rpm[FOC_DYN_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_log_ctrl_fdb_rpm[FOC_DYN_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_log_err_rpm[FOC_DYN_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_log_iq_ref_mA[FOC_DYN_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_dyn_log_iq_mA[FOC_DYN_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_log_current_peak_mA[FOC_DYN_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_dyn_log_fault[FOC_DYN_SPEED_LOG_SIZE];

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_enable = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_reset = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_armed = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_active = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_stop = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_idx = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_decim_ms = 2U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_trigger_rpm = 300U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_detail_log_trigger_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_zero_window_enable = 1U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_zero_post_ms = 300U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_zero_event_idx = 0xFFFFU;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_detail_log_zero_event_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_zero_window_done = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_detail_log_t_ms[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_raw_ref_rpm[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_ref_rpm[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_fdb_rpm[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_ctrl_fdb_rpm[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_iq_ref_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_id_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_iq_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_current_peak_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_hall_raw[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_hall_sector[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_sector_no_change_count[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_theta_hall[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_theta_pred[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_theta_ctrl[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_duty_a[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_duty_b[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_duty_c[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_zero_soft_scale_percent[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_zero_soft_limited_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_zero_soft_active[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_zero_transfer_state[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_zero_signed_iq_cmd_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_zero_iq_ff_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_zero_ctrl_iq_raw_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_zero_pid_freeze_active[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int8_t   g_foc_detail_log_zero_direction_pending[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_decel_hold_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_detail_log_edge_elapsed_us[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_fault[FOC_DETAIL_LOG_SIZE];

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_speed_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_speed_step_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_speed_reset_stats = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_speed_period_ms = 8000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_speed_max_rpm = 4000U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_speed_slew_enable = 1U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_speed_slew_rpm_per_s = 2200U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_speed_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_speed_phase_u16 = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_bidir_speed_raw_ref_rpm = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_bidir_speed_ref_rpm = 0;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_zero_transfer_enable = FOC_BIDIR_ZERO_TRANSFER_ENABLE;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_zero_transfer_state = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_transfer_enter_rpm = FOC_BIDIR_ZERO_TRANSFER_ENTER_RPM;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_transfer_exit_rpm = FOC_BIDIR_ZERO_TRANSFER_EXIT_RPM;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_transfer_ms = FOC_BIDIR_ZERO_TRANSFER_MS;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_relaunch_ms = FOC_BIDIR_ZERO_RELAUNCH_MS;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_zero_relaunch_edge_max_us = FOC_BIDIR_ZERO_RELAUNCH_EDGE_MAX_US;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_zero_hold_iq_mA = FOC_BIDIR_ZERO_HOLD_IQ_MA;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_zero_breakaway_iq_mA = FOC_BIDIR_ZERO_BREAKAWAY_IQ_MA;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_iq_slew_mA_per_s = FOC_BIDIR_ZERO_IQ_SLEW_MA_PER_S;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_zero_signed_iq_cmd_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_zero_iq_ff_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_zero_ctrl_iq_raw_mA = 0;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_zero_pid_freeze_active = 0U;
FOC_AI_DEBUG_ROOT volatile int8_t   g_foc_zero_direction_pending = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_transfer_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_zero_edge_elapsed_us = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_zero_transfer_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_zero_transfer_start_reason = 0U;
FOC_AI_DEBUG_ROOT volatile int8_t   g_foc_zero_transfer_raw_sign = 0;
FOC_AI_DEBUG_ROOT volatile int8_t   g_foc_zero_transfer_prev_sign = 0;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_zero_transfer_raw_decreasing = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_zero_transfer_cmd_decreasing = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_zero_transfer_decel_to_zero = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_transfer_raw_abs_rpm = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_transfer_prev_cmd_abs_rpm = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_zero_cross_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_speed_rpm = 120U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_confirm_ms = 50U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_hold_ms = 80U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_timeout_ms = 600U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_approach_start_rpm = 900U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_approach_slew_rpm_per_s = 1200U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_bidir_zero_approach_brake_limit_mA = 300;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_tail_start_rpm = 500U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_tail_slew_rpm_per_s = 700U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_bidir_zero_tail_brake_limit_mA = 150;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_zero_tail_active = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_tail_fdb_drop_rpm = 120U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_tail_fdb_lead_rpm = 120U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_zero_tail_fdb_catch_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_tail_min_drive_mA = 700U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_tail_drive_deadband_rpm = 80U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_zero_tail_drive_assist_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_zero_cross_state = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_zero_cross_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_cross_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_below_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_zero_approach_active = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_bidir_zero_ref_rpm = 0;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_zero_brake_limited_count = 0U;

/* LiveWatch: 0 stop, 1 fixed, 2 +1000..+4000 sine,
 * 3 +/-1000 smooth bidir, 4 -1000..-4000 sine.
 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_select = FOC_TEST_CASE_STOP;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_applied = FOC_TEST_CASE_STOP;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_last_error = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_test_case_exec_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_motor_id = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_motor_id_applied = 0U;

static uint8_t s_foc_test_case_last_select = FOC_TEST_CASE_STOP;
static uint8_t s_foc_test_case_last_motor_id = 0U;

#define FOC_EXT_API_TEST_LOG_SIZE       24U

#define FOC_EXT_API_ID_ENABLE           1U
#define FOC_EXT_API_ID_DISABLE          2U
#define FOC_EXT_API_ID_SET_CURRENT      3U
#define FOC_EXT_API_ID_SET_VOLTAGE      4U
#define FOC_EXT_API_ID_SET_HYBRID_SPEED 5U
#define FOC_EXT_API_ID_SET_HYBRID_IQ    6U
#define FOC_EXT_API_ID_SET_SPEED        7U
#define FOC_EXT_API_ID_SET_TORQUE       8U
#define FOC_EXT_API_ID_SET_IF           9U
#define FOC_EXT_API_ID_SET_VF           10U
#define FOC_EXT_API_ID_GET_ANGLE_SPEED  11U
#define FOC_EXT_API_ID_GET_FULL         12U
#define FOC_EXT_API_ID_GET_MOTOR_NUM    13U
#define FOC_EXT_API_ID_READ_HALL        14U
#define FOC_EXT_API_ID_WRITE_HALL       15U

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_ext_api_test_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_ext_api_test_done = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_ext_api_test_step = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_ext_api_test_idx = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_ext_api_test_overflow = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_ext_api_test_call_disable = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_ext_api_test_motor_id = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_ext_api_test_skipped_mask = 0x0003U;
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_test_speed_rpm = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_test_id_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_test_iq_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_test_vd_v = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_test_vq_v = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_test_torque_nm = 0.0f;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_ext_api_test_hall_offset = 0;

FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ext_api_log_cb_count[FOC_EXT_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_ext_api_log_api_id[FOC_EXT_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_ext_api_log_result[FOC_EXT_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_log_arg0[FOC_EXT_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_log_arg1[FOC_EXT_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_log_arg2[FOC_EXT_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_log_out0[FOC_EXT_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_log_out1[FOC_EXT_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_ext_api_log_out2[FOC_EXT_API_TEST_LOG_SIZE];

static uint8_t s_foc_ext_api_test_prev_enable = 0U;
#define FOC_SPEED_API_TEST_LOG_SIZE     128U

#define FOC_SPEED_API_PHASE_ENABLE      1U
#define FOC_SPEED_API_PHASE_SET_SPEED   2U
#define FOC_SPEED_API_PHASE_SAMPLE      3U
#define FOC_SPEED_API_PHASE_DISABLE     4U

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_done = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_step = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_idx = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_overflow = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_call_disable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_use_hybrid = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_direction = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_stop_on_fault = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_speed_api_test_motor_id = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_speed_api_test_decim = 100U;
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_test_target_rpm = 0.0f;

FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_speed_api_log_cb_count[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_speed_api_log_phase[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_speed_api_log_result[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_speed_api_log_state[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_speed_api_log_fault[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_speed_api_log_direction[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_log_target_rpm[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_log_ref_rpm[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_log_ctrl_ref_rpm[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_log_fdb_rpm[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_log_ctrl_fdb_rpm[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_log_iq_ref_a[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_log_iq_a[FOC_SPEED_API_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile float    g_foc_speed_api_log_current_peak_a[FOC_SPEED_API_TEST_LOG_SIZE];

static uint8_t s_foc_speed_api_test_prev_enable = 0U;
static uint16_t s_foc_speed_api_test_decim_count = 0U;

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_apply = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_disable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_auto_enable = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_motor_id = 0U;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_id_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_iq_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_current_cmd_result = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_current_cmd_seq = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_current_cmd_state = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_current_cmd_fault = 0U;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_id_ref_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_iq_ref_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_id_fdb_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_iq_fdb_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_current_peak_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_speed_fdb_rpm = 0.0f;

#if 0
static uint8_t s_foc_dyn_speed_prev_enable = 0U;
static uint32_t s_foc_dyn_speed_start_us = 0U;
static uint32_t s_foc_dyn_log_last_us = 0U;
static float s_foc_dyn_abs_err_avg_rpm = 0.0f;

static uint8_t FOC_AI_SpeedNear(float a, float b)
{
  return (FOC_FABS(a - b) < 0.5f) ? 1U : 0U;
}

static int16_t FOC_AI_ToI16(float value, float scale)
{
  float scaled = value * scale;

  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return -32768;
  }

  return (int16_t)scaled;
}

static uint16_t FOC_AI_ToU16(float value, float scale)
{
  float scaled = value * scale;

  if (scaled < 0.0f) {
    return 0U;
  }
  if (scaled > 65535.0f) {
    return 65535U;
  }

  return (uint16_t)scaled;
}

static void FOC_AI_ResetDynamicSpeedLog(void)
{
  uint16_t i;

  g_foc_dyn_log_idx = 0U;
  g_foc_dyn_log_stop = 0U;
  s_foc_dyn_log_last_us = 0U;

  for (i = 0U; i < FOC_DYN_SPEED_LOG_SIZE; i++) {
    g_foc_dyn_log_t_ms[i] = 0U;
    g_foc_dyn_log_ref_rpm[i] = 0;
    g_foc_dyn_log_fdb_rpm[i] = 0;
    g_foc_dyn_log_err_rpm[i] = 0;
    g_foc_dyn_log_iq_ref_mA[i] = 0;
    g_foc_dyn_log_iq_mA[i] = 0;
    g_foc_dyn_log_current_peak_mA[i] = 0U;
    g_foc_dyn_log_fault[i] = 0U;
  }
}

static void FOC_AI_ResetDynamicSpeedStats(uint32_t now_us)
{
  g_foc_dyn_speed_elapsed_ms = 0U;
  g_foc_dyn_speed_phase_u16 = 0U;
  g_foc_dyn_speed_ref_rpm = 0;
  g_foc_dyn_speed_fdb_rpm = 0;
  g_foc_dyn_speed_err_rpm = 0;
  g_foc_dyn_speed_abs_err_rpm = 0U;
  g_foc_dyn_speed_abs_err_avg_rpm = 0U;
  g_foc_dyn_speed_max_abs_err_rpm = 0U;
  g_foc_dyn_speed_iq_ref_mA = 0;
  g_foc_dyn_speed_id_mA = 0;
  g_foc_dyn_speed_iq_mA = 0;
  g_foc_dyn_speed_current_peak_mA = 0U;
  g_foc_dyn_speed_sample_count = 0U;
  s_foc_dyn_abs_err_avg_rpm = 0.0f;
  s_foc_dyn_speed_start_us = now_us;
  FOC_AI_ResetDynamicSpeedLog();
}

static float FOC_AI_CalcDynamicSpeedRef(uint32_t now_us)
{
  uint32_t period_ms = g_foc_dyn_speed_period_ms;
  uint32_t period_us;
  uint32_t elapsed_us;
  uint32_t phase_us;
  float phase;
  float speed_min = (float)g_foc_dyn_speed_min_rpm;
  float speed_max = (float)g_foc_dyn_speed_max_rpm;
  float speed_mid;
  float speed_amp;
  float target;
  float tmp;

  if (period_ms < 100U) {
    period_ms = 100U;
  }
  if (speed_max < speed_min) {
    tmp = speed_max;
    speed_max = speed_min;
    speed_min = tmp;
  }

  period_us = period_ms * 1000U;
  elapsed_us = now_us - s_foc_dyn_speed_start_us;
  phase_us = (period_us > 0U) ? (elapsed_us % period_us) : 0U;
  phase = ((float)phase_us / (float)period_us) * FOC_2PI;

  speed_mid = 0.5f * (speed_min + speed_max);
  speed_amp = 0.5f * (speed_max - speed_min);
  target = speed_mid - speed_amp * FOC_FastCos(phase);

  g_foc_dyn_speed_elapsed_ms = elapsed_us / 1000U;
  g_foc_dyn_speed_phase_u16 = FOC_AI_ToU16(phase, 65535.0f / FOC_2PI);
  g_foc_dyn_speed_ref_rpm = FOC_AI_ToI16(target, 1.0f);

  return target;
}

static void FOC_AI_UpdateDynamicSpeedReference(void)
{
  uint32_t now_us = FOC_HAL_GetTimestampUs();

  if (g_foc_dyn_speed_reset_stats != 0U) {
    g_foc_dyn_speed_reset_stats = 0U;
    FOC_AI_ResetDynamicSpeedStats(now_us);
  }

  if (g_foc_dyn_speed_enable == 0U) {
    s_foc_dyn_speed_prev_enable = 0U;
    return;
  }

  if (s_foc_dyn_speed_prev_enable == 0U) {
    s_foc_dyn_speed_prev_enable = 1U;
    FOC_AI_ResetDynamicSpeedStats(now_us);
  }

  FOC_SetSpeedRef(FOC_AI_CalcDynamicSpeedRef(now_us));
}

static uint8_t FOC_AI_IsDynamicSpeedStartCommand(float speed_ref)
{
  if (FOC_AI_SpeedNear(speed_ref, (float)g_foc_dyn_speed_start_cmd_rpm) != 0U) {
    return 1U;
  }

  if ((g_foc_dyn_speed_start_on_max_ref != 0U) &&
      FOC_AI_SpeedNear(speed_ref, (float)g_foc_dyn_speed_max_rpm) != 0U) {
    return 1U;
  }

  return 0U;
}

static uint8_t FOC_AI_HandleDynamicSpeedExternalRef(float speed_ref)
{
  uint32_t now_us;

  g_foc_dyn_speed_last_ext_ref_rpm = FOC_AI_ToI16(speed_ref, 1.0f);

  if (FOC_AI_IsDynamicSpeedStartCommand(speed_ref) != 0U) {
    if (g_foc_dyn_speed_enable == 0U) {
      now_us = FOC_HAL_GetTimestampUs();
      g_foc_dyn_speed_enable = 1U;
      s_foc_dyn_speed_prev_enable = 1U;
      FOC_AI_ResetDynamicSpeedStats(now_us);
      FOC_SetSpeedRef(FOC_AI_CalcDynamicSpeedRef(now_us));
    }
    return 1U;
  }

  if (g_foc_dyn_speed_enable != 0U) {
    g_foc_dyn_speed_enable = 0U;
    s_foc_dyn_speed_prev_enable = 0U;
    g_foc_dyn_speed_reset_stats = 0U;
  }

  return 0U;
}

static void FOC_AI_PollDynamicSpeedCoreRef(void)
{
  const FOC_Context_t *ctx = FOC_Core_GetContext();
  float speed_ref = ctx->speed_ref;
  uint32_t now_us;

  g_foc_dyn_speed_last_ext_ref_rpm = FOC_AI_ToI16(speed_ref, 1.0f);

  if (FOC_AI_IsDynamicSpeedStartCommand(speed_ref) != 0U) {
    if (g_foc_dyn_speed_enable == 0U) {
      now_us = FOC_HAL_GetTimestampUs();
      g_foc_dyn_speed_enable = 1U;
      s_foc_dyn_speed_prev_enable = 1U;
      FOC_AI_ResetDynamicSpeedStats(now_us);
    }
    return;
  }

  if ((g_foc_dyn_speed_enable != 0U) &&
      (FOC_AI_SpeedNear(speed_ref,
                        (float)g_foc_dyn_speed_ref_rpm) == 0U)) {
    g_foc_dyn_speed_enable = 0U;
    s_foc_dyn_speed_prev_enable = 0U;
    g_foc_dyn_speed_reset_stats = 0U;
  }
}

static void FOC_AI_RecordDynamicSpeedLog(const FOC_Context_t *ctx,
                                         uint32_t now_us,
                                         int16_t err_rpm)
{
  uint16_t idx;
  uint32_t decim_us = (uint32_t)g_foc_dyn_log_decim_ms * 1000U;

  if (g_foc_dyn_log_stop != 0U) {
    return;
  }
  if (decim_us < 1000U) {
    decim_us = 1000U;
  }
  if ((s_foc_dyn_log_last_us != 0U) &&
      ((now_us - s_foc_dyn_log_last_us) < decim_us)) {
    return;
  }

  idx = g_foc_dyn_log_idx;
  if (idx >= FOC_DYN_SPEED_LOG_SIZE) {
    g_foc_dyn_log_stop = 1U;
    return;
  }

  s_foc_dyn_log_last_us = now_us;
  g_foc_dyn_log_t_ms[idx] = g_foc_dyn_speed_elapsed_ms;
  g_foc_dyn_log_ref_rpm[idx] = g_foc_dyn_speed_ref_rpm;
  g_foc_dyn_log_fdb_rpm[idx] = FOC_AI_ToI16(ctx->speed_fdb, 1.0f);
  g_foc_dyn_log_err_rpm[idx] = err_rpm;
  g_foc_dyn_log_iq_ref_mA[idx] = FOC_AI_ToI16(ctx->iq_ref, 1000.0f);
  g_foc_dyn_log_iq_mA[idx] = FOC_AI_ToI16(ctx->i_dq.q, 1000.0f);
  g_foc_dyn_log_current_peak_mA[idx] = FOC_AI_ToU16(ctx->current_peak, 1000.0f);
  g_foc_dyn_log_fault[idx] = (uint16_t)ctx->fault;

  idx++;
  g_foc_dyn_log_idx = idx;
  if (idx >= FOC_DYN_SPEED_LOG_SIZE) {
    g_foc_dyn_log_stop = 1U;
  }
}

static void FOC_AI_UpdateDynamicSpeedMetrics(void)
{
  const FOC_Context_t *ctx;
  uint32_t now_us;
  float err;
  float abs_err;
  int16_t err_rpm;

  if (g_foc_dyn_speed_enable == 0U) {
    return;
  }

  ctx = FOC_Core_GetContext();
  now_us = FOC_HAL_GetTimestampUs();
  err = (float)g_foc_dyn_speed_ref_rpm - ctx->speed_fdb;
  abs_err = FOC_FABS(err);
  err_rpm = FOC_AI_ToI16(err, 1.0f);

  g_foc_dyn_speed_fdb_rpm = FOC_AI_ToI16(ctx->speed_fdb, 1.0f);
  g_foc_dyn_speed_err_rpm = err_rpm;
  g_foc_dyn_speed_abs_err_rpm = FOC_AI_ToU16(abs_err, 1.0f);
  if (g_foc_dyn_speed_abs_err_rpm > g_foc_dyn_speed_max_abs_err_rpm) {
    g_foc_dyn_speed_max_abs_err_rpm = g_foc_dyn_speed_abs_err_rpm;
  }

  s_foc_dyn_abs_err_avg_rpm =
      0.001f * abs_err + 0.999f * s_foc_dyn_abs_err_avg_rpm;
  g_foc_dyn_speed_abs_err_avg_rpm =
      FOC_AI_ToU16(s_foc_dyn_abs_err_avg_rpm, 1.0f);

  g_foc_dyn_speed_iq_ref_mA = FOC_AI_ToI16(ctx->iq_ref, 1000.0f);
  g_foc_dyn_speed_id_mA = FOC_AI_ToI16(ctx->i_dq.d, 1000.0f);
  g_foc_dyn_speed_iq_mA = FOC_AI_ToI16(ctx->i_dq.q, 1000.0f);
  g_foc_dyn_speed_current_peak_mA = FOC_AI_ToU16(ctx->current_peak, 1000.0f);
  g_foc_dyn_speed_sample_count++;

  FOC_AI_RecordDynamicSpeedLog(ctx, now_us, err_rpm);
}
#endif

#define FOC_APP_MOTOR_COUNT    2U
#define FOC_APP_POLE_PAIRS     4U
#define FOC_APP_DIR_NONE       0U
#define FOC_APP_DIR_FORWARD    1U
#define FOC_APP_DIR_REVERSE    2U
#define FOC_APP_MODE_NONE      0U
#define FOC_APP_MODE_SPEED     1U
#define FOC_APP_MODE_CURRENT   3U

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_soft_stop_enable = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_soft_stop_pending[FOC_APP_MOTOR_COUNT] = {0U, 0U};
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_soft_stop_last_motor_id = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_soft_stop_last_reason = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_soft_stop_near_rpm = 80U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_soft_stop_timeout_ms = 2000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_soft_stop_last_result = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_soft_stop_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_soft_stop_timeout_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_soft_stop_elapsed_ms[FOC_APP_MOTOR_COUNT] = {0U, 0U};

static uint32_t s_foc_soft_stop_start_us[FOC_APP_MOTOR_COUNT] = {0U, 0U};

static FocError FOC_AI_CheckMotorId(uint8_t unId)
{
  return (unId < FOC_APP_MOTOR_COUNT) ? FOC_SUCCESS : FOC_MOTOR_ID_INVALID;
}

static FocError FOC_AI_SelectMotor(uint8_t unId)
{
  FocError err = FOC_AI_CheckMotorId(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  FOC_Core_SelectMotor(unId);
  g_foc_selected_motor_id = FOC_HAL_GetSelectedMotor();
  return FOC_SUCCESS;
}

static FocError FOC_AI_MapResult(int result)
{
  if (result == FOC_OK) {
    return FOC_SUCCESS;
  }
  if (result == FOC_BUSY || result == FOC_FAULT) {
    return FOC_MOTOR_DISABLED;
  }
  return FOC_INPUT_PARAMETER_INVALID;
}

static FocError FOC_AI_MapFault(FOC_Fault_e fault)
{
  if (fault == FOC_FAULT_NONE) {
    return FOC_SUCCESS;
  }
  if ((fault & FOC_FAULT_STALL) != 0U) {
    return FOC_MOTOR_STUCKED;
  }
  if ((fault & FOC_FAULT_HALL) != 0U) {
    return FOC_HALL_STATES_INVALID;
  }
  if ((fault & FOC_FAULT_OVERCURRENT) != 0U) {
    return FOC_PHASE_ABC_SQRT_CURRENT_EXCEED;
  }
  if ((fault & FOC_FAULT_OVERVOLTAGE) != 0U) {
    return FOC_PHASE_VOLTAGE_HIGH_EXCEED;
  }
  if ((fault & FOC_FAULT_UNDERVOLTAGE) != 0U) {
    return FOC_PHASE_VOLTAGE_LOW_EXCEED;
  }
  if ((fault & FOC_FAULT_SPEED_DROP) != 0U) {
    return FOC_MOTOR_STUCKED;
  }
  if ((fault & FOC_FAULT_SPEED_FDB_DROP) != 0U) {
    return FOC_ABNORMAL_INTERVAL_TIME;
  }
  return FOC_INPUT_PARAMETER_INVALID;
}

static FocError FOC_AI_EnsureRunningForTarget(float target)
{
  const FOC_Context_t *ctx = FOC_Core_GetContext();

  if (FOC_FABS(target) < 0.0001f) {
    return FOC_SUCCESS;
  }
  if (ctx->state == FOC_STATE_RUNNING) {
    return FOC_SUCCESS;
  }
  if (ctx->state == FOC_STATE_FAULT) {
    return FOC_AI_MapFault(ctx->fault);
  }
  return FOC_AI_MapResult(FOC_Start());
}

static uint8_t FOC_AI_HallRawToU8(const FOC_HallRaw_t *hall_raw)
{
  return (uint8_t)((hall_raw->h1 << 2) | (hall_raw->h2 << 1) | hall_raw->h3);
}

static uint16_t FOC_AI_DirectionToApp(FOC_Dir_e direction)
{
  return (direction == FOC_DIR_CCW) ? FOC_APP_DIR_REVERSE : FOC_APP_DIR_FORWARD;
}

static float FOC_AI_SignedIqRef(const FOC_Context_t *ctx)
{
  return (ctx->direction == FOC_DIR_CCW) ? -ctx->iq_ref : ctx->iq_ref;
}

static FocError FOC_AI_MakeSignedTarget(uint16_t direction,
                                        float magnitude,
                                        float *target)
{
  if (target == NULL) {
    return FOC_POINTER_NULL;
  }
  if (magnitude == 0.0f) {
    *target = 0.0f;
    return FOC_SUCCESS;
  }
  if (direction == FOC_APP_DIR_FORWARD) {
    *target = magnitude;
    return FOC_SUCCESS;
  }
  if (direction == FOC_APP_DIR_REVERSE) {
    *target = -magnitude;
    return FOC_SUCCESS;
  }
  return FOC_INVALID_DIRECITON;
}

static void FOC_AI_ClearAutoModes(void)
{
  speed_ref = -1.0f;
  g_foc_dyn_speed_enable = 0U;
  g_foc_dyn_speed_reverse = 0U;
  g_foc_dyn_speed_reset_stats = 0U;
  g_foc_bidir_speed_enable = 0U;
  g_foc_bidir_speed_step_enable = 0U;
  g_foc_bidir_speed_reset_stats = 0U;
}

static void FOC_AI_SoftStopCancel(uint8_t unId)
{
  if (unId >= FOC_APP_MOTOR_COUNT) {
    return;
  }

  g_foc_soft_stop_pending[unId] = 0U;
  g_foc_soft_stop_elapsed_ms[unId] = 0U;
}

static FocError FOC_AI_RequestSoftStop(uint8_t unId)
{
  const FOC_Context_t *ctx;
  FocError err = FOC_AI_SelectMotor(unId);
  int result;

  if (err != FOC_SUCCESS) {
    return err;
  }

  ctx = FOC_Core_GetContext();
  FOC_AI_ClearAutoModes();

  if (g_foc_soft_stop_enable == 0U) {
    FOC_AI_SoftStopCancel(unId);
    return FOC_AI_MapResult(FOC_Stop());
  }

  if (ctx->state == FOC_STATE_FAULT) {
    FOC_AI_SoftStopCancel(unId);
    return FOC_AI_MapFault(ctx->fault);
  }

  if (ctx->state != FOC_STATE_RUNNING) {
    FOC_AI_SoftStopCancel(unId);
    return FOC_SUCCESS;
  }

  result = FOC_SetSpeedRef(0.0f);
  if (result != FOC_OK) {
    FOC_AI_SoftStopCancel(unId);
    return FOC_AI_MapResult(result);
  }

  if (g_foc_soft_stop_pending[unId] == 0U) {
    s_foc_soft_stop_start_us[unId] = FOC_HAL_GetTimestampUs();
    g_foc_soft_stop_elapsed_ms[unId] = 0U;
  }

  g_foc_soft_stop_pending[unId] = 1U;
  g_foc_soft_stop_last_motor_id = unId;
  g_foc_soft_stop_last_reason = 0U;
  g_foc_soft_stop_last_result = FOC_SUCCESS;
  return FOC_SUCCESS;
}

static void FOC_AI_SoftStopService(void)
{
  uint8_t previous_motor = FOC_Core_GetSelectedMotor();
  uint8_t motor;
  uint32_t now_us = FOC_HAL_GetTimestampUs();
  uint16_t near_rpm = g_foc_soft_stop_near_rpm;
  uint16_t timeout_ms = g_foc_soft_stop_timeout_ms;

  if (near_rpm == 0U) {
    near_rpm = 1U;
  }

  for (motor = 0U; motor < FOC_APP_MOTOR_COUNT; motor++) {
    const FOC_Context_t *ctx;
    uint32_t elapsed_ms;
    uint8_t near_zero;
    uint8_t timed_out;

    if (g_foc_soft_stop_pending[motor] == 0U) {
      continue;
    }

    ctx = FOC_Core_GetContextByMotor(motor);
    if ((ctx->state == FOC_STATE_IDLE) || (ctx->state == FOC_STATE_FAULT)) {
      FOC_AI_SoftStopCancel(motor);
      continue;
    }

    if (ctx->state != FOC_STATE_RUNNING) {
      continue;
    }

    elapsed_ms = (now_us - s_foc_soft_stop_start_us[motor]) / 1000U;
    g_foc_soft_stop_elapsed_ms[motor] = elapsed_ms;

    near_zero =
        ((FOC_FABS(ctx->speed_ref_ctrl) <= (float)near_rpm) &&
         (FOC_FABS(ctx->speed_fdb) <= (float)near_rpm))
        ? 1U
        : 0U;
    timed_out = ((timeout_ms != 0U) && (elapsed_ms >= (uint32_t)timeout_ms))
              ? 1U
              : 0U;

    if ((near_zero == 0U) && (timed_out == 0U)) {
      continue;
    }

    FOC_Core_SelectMotor(motor);
    g_foc_soft_stop_last_result = (uint16_t)FOC_AI_MapResult(FOC_Stop());
    g_foc_soft_stop_pending[motor] = 0U;
    g_foc_soft_stop_last_motor_id = motor;
    g_foc_soft_stop_last_reason = (timed_out != 0U) ? 2U : 1U;
    g_foc_soft_stop_count++;
    if (timed_out != 0U) {
      g_foc_soft_stop_timeout_count++;
    }
  }

  FOC_Core_SelectMotor(previous_motor);
}

static void FOC_SpeedApiTest_ClearLog(void)
{
  uint16_t i;

  g_foc_speed_api_test_done = 0U;
  g_foc_speed_api_test_step = 0U;
  g_foc_speed_api_test_idx = 0U;
  g_foc_speed_api_test_overflow = 0U;
  s_foc_speed_api_test_decim_count = 0U;

  for (i = 0U; i < FOC_SPEED_API_TEST_LOG_SIZE; i++) {
    g_foc_speed_api_log_cb_count[i] = 0U;
    g_foc_speed_api_log_phase[i] = 0U;
    g_foc_speed_api_log_result[i] = 0U;
    g_foc_speed_api_log_state[i] = 0U;
    g_foc_speed_api_log_fault[i] = 0U;
    g_foc_speed_api_log_direction[i] = 0U;
    g_foc_speed_api_log_target_rpm[i] = 0.0f;
    g_foc_speed_api_log_ref_rpm[i] = 0.0f;
    g_foc_speed_api_log_ctrl_ref_rpm[i] = 0.0f;
    g_foc_speed_api_log_fdb_rpm[i] = 0.0f;
    g_foc_speed_api_log_ctrl_fdb_rpm[i] = 0.0f;
    g_foc_speed_api_log_iq_ref_a[i] = 0.0f;
    g_foc_speed_api_log_iq_a[i] = 0.0f;
    g_foc_speed_api_log_current_peak_a[i] = 0.0f;
  }
}

static void FOC_SpeedApiTest_Record(uint16_t phase, FocError result)
{
  const FOC_Context_t *ctx = FOC_Core_GetContext();
  uint8_t idx = g_foc_speed_api_test_idx;

  if (idx >= FOC_SPEED_API_TEST_LOG_SIZE) {
    g_foc_speed_api_test_overflow = 1U;
    return;
  }

  g_foc_speed_api_log_cb_count[idx] = g_foc_ai_callback_count;
  g_foc_speed_api_log_phase[idx] = phase;
  g_foc_speed_api_log_result[idx] = (uint16_t)result;
  g_foc_speed_api_log_state[idx] = (uint16_t)ctx->state;
  g_foc_speed_api_log_fault[idx] = (uint16_t)ctx->fault;
  g_foc_speed_api_log_direction[idx] = (uint16_t)ctx->direction;
  g_foc_speed_api_log_target_rpm[idx] = g_foc_speed_api_test_target_rpm;
  g_foc_speed_api_log_ref_rpm[idx] = ctx->speed_ref;
  g_foc_speed_api_log_ctrl_ref_rpm[idx] = ctx->speed_ref_ctrl;
  g_foc_speed_api_log_fdb_rpm[idx] = ctx->speed_fdb;
  g_foc_speed_api_log_ctrl_fdb_rpm[idx] = ctx->speed_ctrl_fdb;
  g_foc_speed_api_log_iq_ref_a[idx] = ctx->iq_ref;
  g_foc_speed_api_log_iq_a[idx] = ctx->i_dq.q;
  g_foc_speed_api_log_current_peak_a[idx] = ctx->current_peak;
  g_foc_speed_api_test_idx = (uint8_t)(idx + 1U);
}

static void FOC_SpeedApiTest_Finish(void)
{
  FocError result;

  if (g_foc_speed_api_test_call_disable != 0U) {
    result = Foc_DisableFocControl(g_foc_speed_api_test_motor_id);
    if (g_foc_speed_api_test_idx < FOC_SPEED_API_TEST_LOG_SIZE) {
      FOC_SpeedApiTest_Record(FOC_SPEED_API_PHASE_DISABLE, result);
    }
  }

  g_foc_speed_api_test_done = 1U;
  g_foc_speed_api_test_enable = 0U;
}

static void FOC_SpeedApiTest_Service(void)
{
  const FOC_Context_t *ctx;
  FocError result;
  uint16_t decim;
  uint16_t target_rpm_u16;
  uint16_t direction;
  float target_abs;

  if (g_foc_speed_api_test_enable == 0U) {
    s_foc_speed_api_test_prev_enable = 0U;
    return;
  }

  if (s_foc_speed_api_test_prev_enable == 0U) {
    s_foc_speed_api_test_prev_enable = 1U;
    g_foc_ext_api_test_enable = 0U;
    FOC_SpeedApiTest_ClearLog();
  }

  if (g_foc_speed_api_test_done != 0U) {
    return;
  }

  if (g_foc_speed_api_test_step == 0U) {
    result = Foc_EnableFocControl(g_foc_speed_api_test_motor_id);
    FOC_SpeedApiTest_Record(FOC_SPEED_API_PHASE_ENABLE, result);
    g_foc_speed_api_test_step = 1U;
    return;
  }

  if (g_foc_speed_api_test_step == 1U) {
    if (g_foc_speed_api_test_use_hybrid != 0U) {
      target_abs = g_foc_speed_api_test_target_rpm;
      direction = (uint16_t)g_foc_speed_api_test_direction;
      if (target_abs < 0.0f) {
        target_abs = -target_abs;
      }
      if ((direction != FOC_APP_DIR_FORWARD) &&
          (direction != FOC_APP_DIR_REVERSE)) {
        direction = (g_foc_speed_api_test_target_rpm < 0.0f)
                  ? FOC_APP_DIR_REVERSE
                  : FOC_APP_DIR_FORWARD;
      }
      target_rpm_u16 = (target_abs > 65535.0f)
          ? 65535U
          : (uint16_t)target_abs;
      result = Foc_SetHybridControlReference(g_foc_speed_api_test_motor_id,
                                             FOC_APP_MODE_SPEED,
                                             direction,
                                             0U,
                                             0U,
                                             target_rpm_u16,
                                             0U);
    } else {
      result = Foc_SetSpeedReference(g_foc_speed_api_test_motor_id,
                                     g_foc_speed_api_test_target_rpm);
    }
    FOC_SpeedApiTest_Record(FOC_SPEED_API_PHASE_SET_SPEED, result);
    g_foc_speed_api_test_step = 2U;
    return;
  }

  decim = g_foc_speed_api_test_decim;
  if (decim == 0U) {
    decim = 1U;
  }

  s_foc_speed_api_test_decim_count++;
  if (s_foc_speed_api_test_decim_count < decim) {
    return;
  }
  s_foc_speed_api_test_decim_count = 0U;

  FOC_SpeedApiTest_Record(FOC_SPEED_API_PHASE_SAMPLE, FOC_SUCCESS);

  ctx = FOC_Core_GetContext();
  if ((g_foc_speed_api_test_idx >= FOC_SPEED_API_TEST_LOG_SIZE) ||
      ((g_foc_speed_api_test_stop_on_fault != 0U) &&
       ((ctx->state == FOC_STATE_FAULT) || (ctx->fault != FOC_FAULT_NONE)))) {
    FOC_SpeedApiTest_Finish();
  }
}

static void FOC_CurrentCmd_UpdateMonitor(void)
{
  const FOC_Context_t *ctx;
  uint8_t motor_id = g_foc_current_cmd_motor_id;

  ctx = FOC_Core_GetContextByMotor(motor_id);

  g_foc_current_cmd_state = (uint16_t)ctx->state;
  g_foc_current_cmd_fault = (uint16_t)ctx->fault;
  g_foc_current_cmd_id_ref_a = ctx->id_ref;
  g_foc_current_cmd_iq_ref_a = (ctx->direction == FOC_DIR_CCW) ? -ctx->iq_ref : ctx->iq_ref;
  g_foc_current_cmd_id_fdb_a = ctx->i_dq.d;
  g_foc_current_cmd_iq_fdb_a = ctx->i_dq.q;
  g_foc_current_cmd_current_peak_a = ctx->current_peak;
  g_foc_current_cmd_speed_fdb_rpm = ctx->speed_fdb;
}

static void FOC_CurrentCmd_Service(void)
{
  FocError result = FOC_SUCCESS;
  uint8_t motor_id = g_foc_current_cmd_motor_id;
  uint8_t has_cmd = 0U;

  if (g_foc_current_cmd_apply != 0U) {
    g_foc_current_cmd_apply = 0U;
    has_cmd = 1U;

    g_foc_test_case_select = FOC_TEST_CASE_STOP;
    g_foc_speed_api_test_enable = 0U;
    g_foc_ext_api_test_enable = 0U;
    FOC_AI_ClearAutoModes();

    if (g_foc_current_cmd_auto_enable != 0U) {
      result = Foc_EnableFocControl(motor_id);
    }

    if (result == FOC_SUCCESS) {
      result = Foc_SetCurrentReference(motor_id,
                                       g_foc_current_cmd_id_a,
                                       g_foc_current_cmd_iq_a);
    }
  }

  if (g_foc_current_cmd_disable != 0U) {
    g_foc_current_cmd_disable = 0U;
    has_cmd = 1U;
    g_foc_current_cmd_id_a = 0.0f;
    g_foc_current_cmd_iq_a = 0.0f;
    result = Foc_DisableFocControl(motor_id);
  }

  if (has_cmd != 0U) {
    g_foc_current_cmd_result = (uint16_t)result;
    g_foc_current_cmd_seq++;
  }

  FOC_CurrentCmd_UpdateMonitor();
}

static void FOC_ExtApiTest_ClearLog(void)
{
  uint16_t i;

  g_foc_ext_api_test_done = 0U;
  g_foc_ext_api_test_step = 0U;
  g_foc_ext_api_test_idx = 0U;
  g_foc_ext_api_test_overflow = 0U;

  for (i = 0U; i < FOC_EXT_API_TEST_LOG_SIZE; i++) {
    g_foc_ext_api_log_cb_count[i] = 0U;
    g_foc_ext_api_log_api_id[i] = 0U;
    g_foc_ext_api_log_result[i] = 0U;
    g_foc_ext_api_log_arg0[i] = 0.0f;
    g_foc_ext_api_log_arg1[i] = 0.0f;
    g_foc_ext_api_log_arg2[i] = 0.0f;
    g_foc_ext_api_log_out0[i] = 0.0f;
    g_foc_ext_api_log_out1[i] = 0.0f;
    g_foc_ext_api_log_out2[i] = 0.0f;
  }
}

static void FOC_ExtApiTest_Record(uint16_t api_id,
                                  FocError result,
                                  float arg0,
                                  float arg1,
                                  float arg2,
                                  float out0,
                                  float out1,
                                  float out2)
{
  uint8_t idx = g_foc_ext_api_test_idx;

  if (idx >= FOC_EXT_API_TEST_LOG_SIZE) {
    g_foc_ext_api_test_overflow = 1U;
    return;
  }

  g_foc_ext_api_log_cb_count[idx] = g_foc_ai_callback_count;
  g_foc_ext_api_log_api_id[idx] = api_id;
  g_foc_ext_api_log_result[idx] = (uint16_t)result;
  g_foc_ext_api_log_arg0[idx] = arg0;
  g_foc_ext_api_log_arg1[idx] = arg1;
  g_foc_ext_api_log_arg2[idx] = arg2;
  g_foc_ext_api_log_out0[idx] = out0;
  g_foc_ext_api_log_out1[idx] = out1;
  g_foc_ext_api_log_out2[idx] = out2;
  g_foc_ext_api_test_idx = (uint8_t)(idx + 1U);
}

static void FOC_ExtApiTest_Service(void)
{
  const FOC_Context_t *ctx;
  MotorFullStates full_states;
  FocError result = FOC_SUCCESS;
  uint8_t motor_id = g_foc_ext_api_test_motor_id;
  uint8_t motor_num = 0U;
  int16_t hall_offset = -32768;
  float theta = 0.0f;
  float speed = 0.0f;
  float speed_ref = g_foc_ext_api_test_speed_rpm;
  float id_ref = g_foc_ext_api_test_id_a;
  float iq_ref = g_foc_ext_api_test_iq_a;
  float vd_ref = g_foc_ext_api_test_vd_v;
  float vq_ref = g_foc_ext_api_test_vq_v;
  float torque_ref = g_foc_ext_api_test_torque_nm;
  uint16_t hybrid_speed = (speed_ref > 0.0f) ? (uint16_t)speed_ref : 0U;
  uint16_t hybrid_iq_mA = (iq_ref > 0.0f) ? (uint16_t)(iq_ref * 1000.0f) : 0U;

  memset(&full_states, 0, sizeof(full_states));

  if (g_foc_ext_api_test_enable == 0U) {
    s_foc_ext_api_test_prev_enable = 0U;
    return;
  }

  if (s_foc_ext_api_test_prev_enable == 0U) {
    s_foc_ext_api_test_prev_enable = 1U;
    FOC_ExtApiTest_ClearLog();
  }

  if (g_foc_ext_api_test_done != 0U) {
    return;
  }

  switch (g_foc_ext_api_test_step) {
  case 0U:
    result = Foc_EnableFocControl(motor_id);
    ctx = FOC_Core_GetContext();
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_ENABLE, result,
                          (float)motor_id, 0.0f, 0.0f,
                          (float)ctx->state, (float)ctx->fault, 0.0f);
    break;

  case 1U:
    result = Foc_SetSpeedReference(motor_id, speed_ref);
    ctx = FOC_Core_GetContext();
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_SET_SPEED, result,
                          (float)motor_id, speed_ref, 0.0f,
                          ctx->speed_ref, (float)ctx->direction, 0.0f);
    break;

  case 2U:
    result = Foc_SetCurrentReference(motor_id, id_ref, iq_ref);
    ctx = FOC_Core_GetContext();
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_SET_CURRENT, result,
                          (float)motor_id, id_ref, iq_ref,
                          ctx->id_ref, ctx->iq_ref, 0.0f);
    break;

  case 3U:
    result = Foc_SetHybridControlReference(motor_id, FOC_APP_MODE_SPEED,
                                           FOC_APP_DIR_FORWARD, 0U, 0U,
                                           hybrid_speed, 0U);
    ctx = FOC_Core_GetContext();
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_SET_HYBRID_SPEED, result,
                          (float)motor_id, (float)hybrid_speed, 0.0f,
                          ctx->speed_ref, (float)ctx->direction, 0.0f);
    break;

  case 4U:
    result = Foc_SetHybridControlReference(motor_id, FOC_APP_MODE_CURRENT,
                                           FOC_APP_DIR_FORWARD, 0U, 0U,
                                           0U, hybrid_iq_mA);
    ctx = FOC_Core_GetContext();
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_SET_HYBRID_IQ, result,
                          (float)motor_id, (float)hybrid_iq_mA, 0.0f,
                          ctx->id_ref, ctx->iq_ref, 0.0f);
    break;

  case 5U:
    result = Foc_SetVoltageReference(motor_id, vd_ref, vq_ref);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_SET_VOLTAGE, result,
                          (float)motor_id, vd_ref, vq_ref,
                          0.0f, 0.0f, 0.0f);
    break;

  case 6U:
    result = Foc_SetTorqueReference(motor_id, torque_ref);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_SET_TORQUE, result,
                          (float)motor_id, torque_ref, 0.0f,
                          0.0f, 0.0f, 0.0f);
    break;

  case 7U:
    result = Foc_SetIFReference(motor_id, iq_ref, speed_ref);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_SET_IF, result,
                          (float)motor_id, iq_ref, speed_ref,
                          0.0f, 0.0f, 0.0f);
    break;

  case 8U:
    result = Foc_SetVFReference(motor_id, vq_ref, speed_ref);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_SET_VF, result,
                          (float)motor_id, vq_ref, speed_ref,
                          0.0f, 0.0f, 0.0f);
    break;

  case 9U:
    result = Foc_GetAngleAndSpeed(motor_id, &theta, &speed);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_GET_ANGLE_SPEED, result,
                          (float)motor_id, 0.0f, 0.0f,
                          theta, speed, 0.0f);
    break;

  case 10U:
    result = Foc_GetMotorFullParameters(motor_id, &full_states);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_GET_FULL, result,
                          (float)motor_id, 0.0f, 0.0f,
                          full_states.fThetaElec,
                          full_states.fSpeedMechEstimate,
                          (float)full_states.enFocState);
    break;

  case 11U:
    result = Foc_GetMotorNum(0U, 0U, motor_id, &motor_num);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_GET_MOTOR_NUM, result,
                          (float)motor_id, 0.0f, 0.0f,
                          (float)motor_num, 0.0f, 0.0f);
    break;

  case 12U:
    result = Foc_ReadMotorHallStates(motor_id, &hall_offset);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_READ_HALL, result,
                          (float)motor_id, 0.0f, 0.0f,
                          (float)hall_offset, 0.0f, 0.0f);
    break;

  case 13U:
    hall_offset = g_foc_ext_api_test_hall_offset;
    result = Foc_WriteMotorHallStates(motor_id, &hall_offset,
                                      g_foc_ext_api_test_hall_offset);
    FOC_ExtApiTest_Record(FOC_EXT_API_ID_WRITE_HALL, result,
                          (float)motor_id,
                          (float)g_foc_ext_api_test_hall_offset,
                          0.0f,
                          (float)hall_offset, 0.0f, 0.0f);
    break;

  case 14U:
    if (g_foc_ext_api_test_call_disable != 0U) {
      result = Foc_DisableFocControl(motor_id);
      ctx = FOC_Core_GetContext();
      FOC_ExtApiTest_Record(FOC_EXT_API_ID_DISABLE, result,
                            (float)motor_id, 0.0f, 0.0f,
                            (float)ctx->state, (float)ctx->fault, 0.0f);
    }
    break;

  default:
    g_foc_ext_api_test_done = 1U;
    g_foc_ext_api_test_enable = 0U;
    return;
  }

  g_foc_ext_api_test_step++;
  if (g_foc_ext_api_test_step > 14U) {
    g_foc_ext_api_test_done = 1U;
    g_foc_ext_api_test_enable = 0U;
  }
}

void Foc_AlgorithmControlCallback_AI(void);
void Foc_Init_AI(void);
FocError Foc_EnableFocControl_AI(uint8_t unId);
FocError Foc_DisableFocControl_AI(uint8_t unId);
FocError Foc_SetCurrentReference_AI(uint8_t unId, float fId, float fIq);
FocError Foc_SetHybridControlReference_AI(uint8_t unId, uint8_t unMode, uint16_t unParam1, uint16_t unParam2,
                                          uint16_t unParam3, uint16_t unParam4, uint16_t unParam5);
FocError Foc_SetSpeedReference_AI(uint8_t unId, float fSpeed);
FocError Foc_GetMotorFullParameters_AI(uint8_t unId, MotorFullStates *pstMotorFullStates);
FocError Foc_GetMotorNum_AI(uint8_t unCarConfigID, uint8_t unSeatID, uint8_t unMotorID, uint8_t *punMotorNum);
FocError Foc_SetVoltageReference_AI(uint8_t unId, float fVd, float fVq);
FocError Foc_SetTorqueReference_AI(uint8_t unId, float fTorque);
FocError Foc_SetIFReference_AI(uint8_t unId, float fIq, float fSpeed);
FocError Foc_SetVFReference_AI(uint8_t unId, float fVq, float fSpeed);
FocError Foc_GetAngleAndSpeed_AI(uint8_t unId, float *pfThetaElec, float *pfSpeed);
FocError Foc_ReadMotorHallStates_AI(uint8_t unId, int16_t *pstHallStatesOffset);
FocError Foc_WriteMotorHallStates_AI(uint8_t unId, int16_t *pstHallStatesOffset, int16_t nHallDistanceOffset);
static void FOC_TestCase_Service(void);

/*******************************************************************************************

  函数名称:  Foc_AlgorithmControlCallback

  函数功能:  电机的Foc中断函数，基于电流环的控制目标完成三相电机占空比输出

  输入参数：  无

  输出参数： 无

  返回值:    无

 *******************************************************************************************/

void Foc_AlgorithmControlCallback_AI(void){

  g_foc_ai_callback_count++;

  FOC_TestCase_Service();

  FOC_CurrentCmd_Service();

  FOC_MainLoop();

  FOC_AI_SoftStopService();

  FOC_CurrentCmd_UpdateMonitor();

  FOC_SpeedApiTest_Service();

  FOC_ExtApiTest_Service();

}

 

/*******************************************************************************************

  函数名称:  Foc_Init

  函数功能:   FOC电流环控制模块初始化，主要是配置霍尔中断回调函数，使用FOC模块提供的一切API接口前必须先被执行

  输入参数： 无

  输出参数： 无

  返回值:    无

 *******************************************************************************************/

void Foc_Init_AI(void)
{
  FOC_Config_t config;

  memset(&config, 0, sizeof(FOC_Config_t));

  config.motor.pole_pairs = FOC_APP_POLE_PAIRS;
  config.motor.rs = 0.65f;
  config.motor.ls_d = 0.00007f;
  config.motor.ls_q = 0.00007f;
  config.motor.v_bus = 12.0f;
  config.motor.max_speed_rpm = 4000.0f;
  config.motor.max_current_a = 5.0f;

  config.current_d_pid.kp = 0.25f;
  config.current_d_pid.ki = 60.0f;
  config.current_d_pid.kd = 0.0f;
  config.current_d_pid.out_max = 10.0f;
  config.current_d_pid.out_min = -10.0f;

  config.current_q_pid.kp = 0.25f;
  config.current_q_pid.ki = 60.0f;
  config.current_q_pid.kd = 0.0f;
  config.current_q_pid.out_max = 10.0f;
  config.current_q_pid.out_min = -10.0f;

  config.speed_pid.kp = 0.0065f;
  config.speed_pid.ki = 0.0008f;
  config.speed_pid.kd = 0.0f;
  config.speed_pid.out_max = config.motor.max_current_a;
  config.speed_pid.out_min = -config.motor.max_current_a;

  (void)FOC_Init(&config);
}

 

/*******************************************************************************************

  函数名称:  Foc_EnableFocControl

  函数功能:  激活电机的FOC电流环控制输出

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_EnableFocControl_AI(uint8_t unId)
{
  const FOC_Context_t *ctx;
  FocError err = FOC_AI_SelectMotor(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  ctx = FOC_Core_GetContext();
  if (ctx->state == FOC_STATE_RUNNING) {
    FOC_AI_SoftStopCancel(unId);
    return FOC_SUCCESS;
  }
  if (ctx->state == FOC_STATE_FAULT) {
    return FOC_AI_MapFault(ctx->fault);
  }

  FOC_AI_SoftStopCancel(unId);
  return FOC_AI_MapResult(FOC_Start());
}

 

/*******************************************************************************************

  函数名称:  Foc_DisableFocControl

  函数功能:  停止电机的FOC电流环控制输出

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_DisableFocControl_AI(uint8_t unId)
{
  FocError err = FOC_AI_SelectMotor(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  FOC_AI_SoftStopCancel(unId);
  FOC_AI_ClearAutoModes();
  return FOC_AI_MapResult(FOC_Stop());
}

/*******************************************************************************************

  函数名称:  Foc_SetCurrentReference

  函数功能:  设置电机以电流控制闭环控制运行，d，q轴目标参考电流

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

            fId：d轴目标参考电流，单位A

            fIq：q轴目标参考电流，单位A

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_SetCurrentReference_AI(uint8_t unId, float fId, float fIq)
{
  FocError err = FOC_AI_SelectMotor(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_EnsureRunningForTarget((FOC_FABS(fId) > FOC_FABS(fIq)) ? fId : fIq);
  if (err != FOC_SUCCESS) {
    return err;
  }

  FOC_AI_SoftStopCancel(unId);
  FOC_AI_ClearAutoModes();
  return FOC_AI_MapResult(FOC_SetCurrentRef(fId, fIq));
}

 

/*******************************************************************************************

  函数名称:  Foc_SetHybridControlReference

  函数功能:  无刷电机混合控制接口

   *******************************************************************************************/

FocError Foc_SetHybridControlReference_AI(uint8_t unId, uint8_t unMode, uint16_t unParam1, uint16_t unParam2,
                                       uint16_t unParam3, uint16_t unParam4, uint16_t unParam5)
{
  float target;
  FocError err;

  (void)unParam2;
  (void)unParam3;

  if (unMode == FOC_APP_MODE_NONE) {
    return FOC_AI_RequestSoftStop(unId);
  }

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  if (unMode == FOC_APP_MODE_SPEED) {
    err = FOC_AI_MakeSignedTarget(unParam1, (float)unParam4, &target);
    if (err != FOC_SUCCESS) {
      return err;
    }
    return Foc_SetSpeedReference_AI(unId, target);
  }

  if (unMode == FOC_APP_MODE_CURRENT) {
    err = FOC_AI_MakeSignedTarget(unParam1, (float)unParam5 * 0.001f, &target);
    if (err != FOC_SUCCESS) {
      return err;
    }
    return Foc_SetCurrentReference_AI(unId, 0.0f, target);
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

 

/*******************************************************************************************

  函数名称:  Foc_SetSpeedReference

  函数功能:  设置速度闭环运行电机

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

            fSpeed：电机目标转速，RPM单位

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_SetSpeedReference_AI(uint8_t unId, float fSpeed)
{
  FocError err;

  if (FOC_FABS(fSpeed) < 0.0001f) {
    return FOC_AI_RequestSoftStop(unId);
  }

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_EnsureRunningForTarget(fSpeed);
  if (err != FOC_SUCCESS) {
    return err;
  }

  FOC_AI_SoftStopCancel(unId);
  FOC_AI_ClearAutoModes();
  return FOC_AI_MapResult(FOC_SetSpeedRef(fSpeed));
}

 

/*******************************************************************************************

  函数名称:  Foc_GetMotorFullParameters

  函数功能:  获取电机当前状态

  输入参数: unId：设备号，0：水平无刷电机  1：坐盆无刷电机

  输出参数: pstMotorFullStates：当前电机状态的所有参数的结构体指针

  返回值:  FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_GetMotorFullParameters_AI(uint8_t unId, MotorFullStates *pstMotorFullStates)
{
  const FOC_Context_t *ctx;
  FocError err;

  if (pstMotorFullStates == NULL) {
    return FOC_POINTER_NULL;
  }

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  ctx = FOC_Core_GetContext();
  memset(pstMotorFullStates, 0, sizeof(*pstMotorFullStates));

  pstMotorFullStates->unId = unId;
  pstMotorFullStates->unHallState = FOC_AI_HallRawToU8(&ctx->hall_raw);
  pstMotorFullStates->fSpeedMechEstimate = ctx->speed_fdb;
  pstMotorFullStates->fSpeedMechEstimateFiltered = ctx->speed_filtered;
  pstMotorFullStates->fThetaElec = ctx->theta_e;
  pstMotorFullStates->fId = ctx->i_dq.d;
  pstMotorFullStates->fIq = ctx->i_dq.q;
  pstMotorFullStates->fIqRef = FOC_AI_SignedIqRef(ctx);
  pstMotorFullStates->fIa = ctx->i_abc.ia;
  pstMotorFullStates->fIb = ctx->i_abc.ib;
  pstMotorFullStates->fIc = ctx->i_abc.ic;
  pstMotorFullStates->fUd = ctx->v_dq.d;
  pstMotorFullStates->fUq = ctx->v_dq.q;
  pstMotorFullStates->unNumberPoles = FOC_APP_POLE_PAIRS;
  pstMotorFullStates->unUpdateTime = (ctx->hall_sector_timestamp_us != 0U)
      ? ctx->hall_sector_timestamp_us
      : FOC_HAL_GetTimestampUs();
  pstMotorFullStates->enFocState = FOC_AI_MapFault(ctx->fault);
  pstMotorFullStates->unHeadIndexHall = -1;
  pstMotorFullStates->unHeadIndexAppHall = -1;
  pstMotorFullStates->unDirection = FOC_AI_DirectionToApp(ctx->direction);

  return FOC_SUCCESS;
}

 

/*******************************************************************************************

  函数名称:  Foc_GetMotorNum

  函数功能:  FOC电机号接口函数

  输入参数: unCarConfigID：车型ID, SNHB暂定为1, 其余未定义

           unSeatID：座椅ID，副驾座椅CPLTSEAT

           unMotorID：电机ID，水平电机：LEVELMOTOR，坐盆电机：BIDETMOTOR

  输出参数: punMotorNum: 电机号指针

  返回值:  FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_GetMotorNum_AI(uint8_t unCarConfigID, uint8_t unSeatID, uint8_t unMotorID, uint8_t *punMotorNum)
{
  (void)unCarConfigID;
  (void)unSeatID;
  if (punMotorNum == NULL) {
    return FOC_POINTER_NULL;
  }

  if (unMotorID >= FOC_APP_MOTOR_COUNT) {
    return FOC_MOTOR_ID_INVALID;
  }

  *punMotorNum = unMotorID;
  return FOC_SUCCESS;
}

 

FocError Foc_SetVoltageReference_AI(uint8_t unId, float fVd, float fVq)
{
  FocError err = FOC_AI_CheckMotorId(unId);

  (void)fVd;
  (void)fVq;

  if (err != FOC_SUCCESS) {
    return err;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

FocError Foc_SetTorqueReference_AI(uint8_t unId, float fTorque)
{
  FocError err = FOC_AI_CheckMotorId(unId);

  (void)fTorque;

  if (err != FOC_SUCCESS) {
    return err;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

FocError Foc_SetIFReference_AI(uint8_t unId, float fIq, float fSpeed)
{
  FocError err = FOC_AI_CheckMotorId(unId);

  (void)fIq;
  (void)fSpeed;

  if (err != FOC_SUCCESS) {
    return err;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

FocError Foc_SetVFReference_AI(uint8_t unId, float fVq, float fSpeed)
{
  FocError err = FOC_AI_CheckMotorId(unId);

  (void)fVq;
  (void)fSpeed;

  if (err != FOC_SUCCESS) {
    return err;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

FocError Foc_GetAngleAndSpeed_AI(uint8_t unId, float *pfThetaElec, float *pfSpeed)
{
  const FOC_Context_t *ctx;
  FocError err;

  if ((pfThetaElec == NULL) || (pfSpeed == NULL)) {
    return FOC_POINTER_NULL;
  }

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  ctx = FOC_Core_GetContext();
  *pfThetaElec = ctx->theta_e;
  *pfSpeed = ctx->speed_fdb;

  return FOC_SUCCESS;
}

FocError Foc_ReadMotorHallStates_AI(uint8_t unId, int16_t *pstHallStatesOffset)
{
  FocError err;

  if (pstHallStatesOffset == NULL) {
    return FOC_POINTER_NULL;
  }

  err = FOC_AI_CheckMotorId(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

FocError Foc_WriteMotorHallStates_AI(uint8_t unId, int16_t *pstHallStatesOffset, int16_t nHallDistanceOffset)
{
  FocError err;

  (void)nHallDistanceOffset;

  if (pstHallStatesOffset == NULL) {
    return FOC_POINTER_NULL;
  }

  err = FOC_AI_CheckMotorId(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}
FOC_AI_DEBUG_ROOT volatile float gfSpeedTarget = 0.0f;

FOC_AI_DEBUG_ROOT volatile uint8_t gunCtrl = 0U;

static float s_foc_test_case_last_fixed_ref = 0.0f;

static uint8_t FOC_TestCase_GetMotorId(void)
{
  return (g_foc_test_motor_id < FOC_APP_MOTOR_COUNT)
       ? g_foc_test_motor_id
       : 0U;
}

static void FOC_TestCase_ClearAutoModes(void)
{
  speed_ref = -1.0f;
  g_foc_dyn_speed_enable = 0U;
  g_foc_dyn_speed_reverse = 0U;
  g_foc_dyn_speed_reset_stats = 0U;
  g_foc_bidir_speed_enable = 0U;
  g_foc_bidir_speed_step_enable = 0U;
  g_foc_bidir_speed_reset_stats = 0U;
}

static float FOC_TestCase_GetFixedSpeedRef(uint8_t unId)
{
  const FOC_Context_t *ctx = FOC_Core_GetContextByMotor(unId);
  float fixed_ref = gfSpeedTarget;

  if (FOC_FABS(fixed_ref) < 0.5f) {
    fixed_ref = ctx->speed_ref;
    if ((fixed_ref > 0.0f) && (ctx->direction == FOC_DIR_CCW)) {
      fixed_ref = -fixed_ref;
    }
  }

  return fixed_ref;
}

static void FOC_TestCase_Apply(uint8_t test_case)
{
  uint8_t unId = FOC_TestCase_GetMotorId();

  g_foc_test_case_last_error = 0U;
  speed_ref = -1.0f;

  if (unId != s_foc_test_case_last_motor_id) {
    FOC_TestCase_ClearAutoModes();
    Foc_DisableFocControl(s_foc_test_case_last_motor_id);
  }

  if(test_case == FOC_TEST_CASE_STOP){

    FOC_TestCase_ClearAutoModes();

    Foc_DisableFocControl(unId);

  }else if(test_case == FOC_TEST_CASE_FIXED_SPEED){

    float fixed_ref = FOC_TestCase_GetFixedSpeedRef(unId);

    FOC_TestCase_ClearAutoModes();
    g_foc_dyn_speed_start_on_max_ref = 0U;
    g_foc_dyn_speed_start_on_max_fdb = 0U;

    Foc_EnableFocControl(unId);

    Foc_SetSpeedReference(unId, fixed_ref);
    s_foc_test_case_last_fixed_ref = fixed_ref;

  }else if(test_case == FOC_TEST_CASE_DYN_SPEED_CW){

    Foc_EnableFocControl(unId);

    g_foc_dyn_speed_min_rpm = 1000U;
    g_foc_dyn_speed_max_rpm = 4000U;
    g_foc_dyn_speed_reverse = 0U;
    g_foc_dyn_speed_start_on_max_ref = 0U;
    g_foc_dyn_speed_start_on_max_fdb = 0U;
    g_foc_bidir_speed_enable = 0U;
    g_foc_bidir_speed_step_enable = 0U;
    g_foc_bidir_speed_reset_stats = 0U;

    g_foc_dyn_speed_enable = 1U;

    g_foc_dyn_speed_reset_stats = 1U;

  }else if(test_case == FOC_TEST_CASE_BIDIR_SWITCH){

    Foc_EnableFocControl(unId);

    g_foc_dyn_speed_enable = 0U;
    g_foc_dyn_speed_reverse = 0U;
    g_foc_dyn_speed_reset_stats = 0U;

    g_foc_bidir_speed_period_ms = 4000U;
    g_foc_bidir_speed_max_rpm = 1000U;
    g_foc_bidir_speed_step_enable = 2U;
    g_foc_bidir_speed_slew_enable = 0U;
    g_foc_bidir_speed_slew_rpm_per_s = 1500U;
    g_foc_zero_transfer_enable = 1U;
    g_foc_zero_transfer_enter_rpm = 160U;
    g_foc_zero_transfer_exit_rpm = 110U;
    g_foc_zero_transfer_ms = 80U;
    g_foc_zero_relaunch_ms = 30U;
    g_foc_zero_relaunch_edge_max_us = 80000U;
    g_foc_zero_hold_iq_mA = 550;
    g_foc_zero_breakaway_iq_mA = 1100;
    g_foc_zero_iq_slew_mA_per_s = 16000U;
    g_foc_zero_transfer_state = 0U;
    g_foc_zero_signed_iq_cmd_mA = 0;
    g_foc_zero_iq_ff_mA = 0;
    g_foc_zero_ctrl_iq_raw_mA = 0;
    g_foc_zero_pid_freeze_active = 0U;
    g_foc_zero_direction_pending = 0;
    g_foc_zero_transfer_elapsed_ms = 0U;
    g_foc_zero_transfer_count = 0U;
    g_foc_zero_transfer_start_reason = 0U;
    g_foc_zero_transfer_raw_sign = 0;
    g_foc_zero_transfer_prev_sign = 0;
    g_foc_zero_transfer_raw_decreasing = 0U;
    g_foc_zero_transfer_cmd_decreasing = 0U;
    g_foc_zero_transfer_decel_to_zero = 0U;
    g_foc_zero_transfer_raw_abs_rpm = 0U;
    g_foc_zero_transfer_prev_cmd_abs_rpm = 0U;
    g_foc_detail_log_enable = 1U;
    g_foc_detail_log_decim_ms = 2U;
    g_foc_detail_log_trigger_rpm = 300U;
    g_foc_detail_log_zero_window_enable = 1U;
    g_foc_detail_log_zero_post_ms = 300U;
    g_foc_detail_log_zero_event_idx = 0xFFFFU;
    g_foc_detail_log_zero_event_count = 0U;
    g_foc_detail_log_zero_window_done = 0U;
    g_foc_detail_log_reset = 1U;
    g_foc_bidir_zero_cross_enable = 0U;
    g_foc_bidir_zero_speed_rpm = 120U;
    g_foc_bidir_zero_confirm_ms = 50U;
    g_foc_bidir_zero_hold_ms = 120U;
    g_foc_bidir_zero_timeout_ms = 1200U;
    g_foc_bidir_zero_approach_start_rpm = 1000U;
    g_foc_bidir_zero_approach_slew_rpm_per_s = 2000U;
    g_foc_bidir_zero_approach_brake_limit_mA = 300;
    g_foc_bidir_zero_tail_start_rpm = 500U;
    g_foc_bidir_zero_tail_slew_rpm_per_s = 700U;
    g_foc_bidir_zero_tail_brake_limit_mA = 150;
    g_foc_bidir_zero_tail_fdb_drop_rpm = 120U;
    g_foc_bidir_zero_tail_fdb_lead_rpm = 120U;
    g_foc_bidir_zero_tail_min_drive_mA = 700U;
    g_foc_bidir_zero_tail_drive_deadband_rpm = 80U;
    g_foc_bidir_speed_enable = 1U;

    g_foc_bidir_speed_reset_stats = 1U;

  }else if(test_case == FOC_TEST_CASE_DYN_SPEED_CCW){

    Foc_EnableFocControl(unId);

    g_foc_dyn_speed_min_rpm = 1000U;
    g_foc_dyn_speed_max_rpm = 4000U;
    g_foc_dyn_speed_reverse = 1U;
    g_foc_dyn_speed_start_on_max_ref = 0U;
    g_foc_dyn_speed_start_on_max_fdb = 0U;
    g_foc_bidir_speed_enable = 0U;
    g_foc_bidir_speed_step_enable = 0U;
    g_foc_bidir_speed_reset_stats = 0U;

    g_foc_dyn_speed_enable = 1U;

    g_foc_dyn_speed_reset_stats = 1U;

  }else{

    g_foc_test_case_last_error = test_case;

    return;

  }

  g_foc_test_case_applied = test_case;
  g_foc_test_motor_id_applied = unId;
  s_foc_test_case_last_motor_id = unId;
  g_foc_test_case_exec_count++;
}
static void FOC_TestCase_Service(void)
{
  uint8_t test_case = g_foc_test_case_select;
  uint8_t motor_id = FOC_TestCase_GetMotorId();

  if((test_case == s_foc_test_case_last_select) &&
     (motor_id == s_foc_test_case_last_motor_id)){
    if (test_case == FOC_TEST_CASE_FIXED_SPEED) {
      float fixed_ref = FOC_TestCase_GetFixedSpeedRef(motor_id);
      if (FOC_FABS(fixed_ref - s_foc_test_case_last_fixed_ref) >= 0.5f) {
        FOC_TestCase_Apply(test_case);
      }
    }
    return;
  }

  s_foc_test_case_last_select = test_case;

  FOC_TestCase_Apply(test_case);
}

void Foc_TestCase(void)
{
  g_foc_test_case_select = gunCtrl;
  s_foc_test_case_last_select = gunCtrl;
  FOC_TestCase_Apply(gunCtrl);
}
void Foc_AlgorithmControlCallback(void)
{
  Foc_AlgorithmControlCallback_AI();
}

void Foc_Init(void)
{
  Foc_Init_AI();
}

FocError Foc_EnableFocControl(uint8_t unId)
{
  return Foc_EnableFocControl_AI(unId);
}

FocError Foc_DisableFocControl(uint8_t unId)
{
  return Foc_DisableFocControl_AI(unId);
}

FocError Foc_SetCurrentReference(uint8_t unId, float fId, float fIq)
{
  return Foc_SetCurrentReference_AI(unId, fId, fIq);
}

FocError Foc_SetHybridControlReference(uint8_t unId, uint8_t unMode, uint16_t unParam1, uint16_t unParam2,
                                       uint16_t unParam3, uint16_t unParam4, uint16_t unParam5)
{
  return Foc_SetHybridControlReference_AI(unId, unMode, unParam1, unParam2,
                                          unParam3, unParam4, unParam5);
}

FocError Foc_SetSpeedReference(uint8_t unId, float fSpeed)
{
  return Foc_SetSpeedReference_AI(unId, fSpeed);
}

FocError Foc_GetMotorFullParameters(uint8_t unId, MotorFullStates *pstMotorFullStates)
{
  return Foc_GetMotorFullParameters_AI(unId, pstMotorFullStates);
}

FocError Foc_GetMotorNum(uint8_t unCarConfigID, uint8_t unSeatID, uint8_t unMotorID, uint8_t *punMotorNum)
{
  return Foc_GetMotorNum_AI(unCarConfigID, unSeatID, unMotorID, punMotorNum);
}

FocError Foc_SetVoltageReference(uint8_t unId, float fVd, float fVq)
{
  return Foc_SetVoltageReference_AI(unId, fVd, fVq);
}

FocError Foc_SetTorqueReference(uint8_t unId, float fTorque)
{
  return Foc_SetTorqueReference_AI(unId, fTorque);
}

FocError Foc_SetIFReference(uint8_t unId, float fIq, float fSpeed)
{
  return Foc_SetIFReference_AI(unId, fIq, fSpeed);
}

FocError Foc_SetVFReference(uint8_t unId, float fVq, float fSpeed)
{
  return Foc_SetVFReference_AI(unId, fVq, fSpeed);
}

FocError Foc_GetAngleAndSpeed(uint8_t unId, float *pfThetaElec, float *pfSpeed)
{
  return Foc_GetAngleAndSpeed_AI(unId, pfThetaElec, pfSpeed);
}

FocError Foc_ReadMotorHallStates(uint8_t unId, int16_t *pstHallStatesOffset)
{
  return Foc_ReadMotorHallStates_AI(unId, pstHallStatesOffset);
}

FocError Foc_WriteMotorHallStates(uint8_t unId, int16_t *pstHallStatesOffset, int16_t nHallDistanceOffset)
{
  return Foc_WriteMotorHallStates_AI(unId, pstHallStatesOffset, nHallDistanceOffset);
}
