
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

/* External接口调试计数：每进入一次FOC周期回调自增一次，用于Watch确认主循环是否在跑。 */
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_count = 0U;
/* 最近一次通过External接口选中的FOC物理电机号，底层只支持0/1。 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_selected_motor_id = 0U;
/* 应用层“水平电机”的子电机ID，Foc_GetMotorNum用它匹配应用层传入的unMotorID。 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_level_motor_id = 1U;
/* 应用层“坐盆电机”的子电机ID，当前应用层坐盆默认是5。 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_bidet_motor_id = 5U;
/* 应用层水平电机映射到的FOC物理电机号，默认0。 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_level_foc_motor_id = 0U;
/* 应用层坐盆电机映射到的FOC物理电机号，默认1。 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_bidet_foc_motor_id = 1U;
/* 最近一次Foc_GetMotorNum收到的应用层电机ID，便于定位映射输入。 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_last_get_motor_num_app_id = 0U;
/* 最近一次Foc_GetMotorNum输出的FOC物理电机号；0xFF表示本次未成功输出。 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_last_get_motor_num_foc_id = 0xFFU;
/* 最近一次Foc_GetMotorNum返回的错误码，Watch中用于确认映射是否成功。 */
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_last_get_motor_num_err = FOC_SUCCESS;
/* 动态速度测试使用的外部标志，External接口清自动模式时会间接影响相关测试模式。 */
extern volatile uint8_t g_foc_dyn_speed_start_on_max_fdb;
/* 旧测试入口的速度给定，External正式控制前会置为-1以退出旧测试给定。 */
extern volatile float speed_ref;

#define FOC_DYN_SPEED_LOG_SIZE 512U
#define FOC_DETAIL_LOG_SIZE    512U

#define FOC_TEST_CASE_STOP              0U
#define FOC_TEST_CASE_FIXED_SPEED       1U
#define FOC_TEST_CASE_DYN_SPEED_CW      2U
#define FOC_TEST_CASE_BIDIR_SWITCH      3U
#define FOC_TEST_CASE_DYN_SPEED_CCW     4U
#define FOC_TEST_CASE_CRADLE_SEAT       5U

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
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_zero_handoff_ms = FOC_BIDIR_ZERO_HANDOFF_MS;
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
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_zero_relaunch_abort_count = 0U;
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
 * 3 +/-1000 sine, 4 -1000..-4000 sine, 5 seat cradle.
 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_select = FOC_TEST_CASE_STOP;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_applied = FOC_TEST_CASE_STOP;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_last_error = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_test_case_exec_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_motor_id = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_motor_id_applied = 0U;

FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_amplitude_hall = 120U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_max_rpm = 300U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_kp_rpm_per_hall = 6U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_slow_zone_hall = 45U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_min_move_rpm = 45U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_slew_rpm_per_s = 450U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_deadband_hall = 2U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_hold_ms = 300U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_control_ms = 5U;
FOC_AI_DEBUG_ROOT volatile int8_t   g_foc_cradle_cmd_polarity = 1;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_cradle_reset_center = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_cradle_active = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_cradle_hold_active = 0U;
FOC_AI_DEBUG_ROOT volatile int8_t   g_foc_cradle_target_side = 1;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_cradle_center_hall = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_cradle_pos_hall = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_cradle_target_hall = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_cradle_error_hall = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_cradle_abs_error_hall = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_cradle_speed_ref_rpm = 0;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_cradle_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_cradle_cycle_count = 0U;

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

/* External接口允许的应用层ID上限；注意FOC核心物理电机仍只有0/1。 */
#define FOC_APP_MOTOR_COUNT    200U
/* FOC核心当前支持的物理电机数量。 */
#define FOC_PHY_MOTOR_COUNT    2U
/* 当前FOC初始化使用的电机极对数。 */
#define FOC_APP_POLE_PAIRS     4U
/* 应用层方向编码：无方向，仅目标为0时允许。 */
#define FOC_APP_DIR_NONE       0U
/* 应用层方向编码：正方向，转换后目标值为正。 */
#define FOC_APP_DIR_FORWARD    1U
/* 应用层方向编码：反方向，转换后目标值为负。 */
#define FOC_APP_DIR_REVERSE    2U
/* Hybrid模式0：头文件语义为Vq比例+速度，当前实现等同速度闭环。 */
#define FOC_APP_MODE_VQ_RATIO_SPEED    0U
/* Hybrid模式1：速度闭环，使用unParam4作为rpm目标。 */
#define FOC_APP_MODE_SPEED             1U
/* Hybrid模式2：头文件语义为Vq比例+电流，当前实现等同电流闭环。 */
#define FOC_APP_MODE_VQ_RATIO_CURRENT  2U
/* Hybrid模式3：电流闭环，使用unParam5作为mA目标。 */
#define FOC_APP_MODE_CURRENT           3U
/* Hybrid模式4：Vq目标，当前会进入未完整实现的电压接口。 */
#define FOC_APP_MODE_VQ                4U

static FocError FOC_AI_CheckMotorId(uint8_t unId)
{
  return (unId < FOC_PHY_MOTOR_COUNT) ? FOC_SUCCESS : FOC_MOTOR_ID_INVALID;
}

static FocError FOC_AI_CheckPhysicalMotorId(uint8_t unId)
{
  return (unId < FOC_PHY_MOTOR_COUNT) ? FOC_SUCCESS : FOC_MOTOR_ID_INVALID;
}

static FocError FOC_AI_MapAppMotorNum(uint8_t unMotorID, uint8_t *punMotorNum)
{
  /* 应用层电机ID映射后的FOC物理电机号，成功时只能是0或1。 */
  uint8_t mapped_motor;

  if (punMotorNum == NULL) {
    return FOC_POINTER_NULL;
  }

  if (unMotorID >= FOC_APP_MOTOR_COUNT) {
    return FOC_MOTOR_ID_INVALID;
  }

  if (unMotorID == g_foc_app_level_motor_id) {
    mapped_motor = g_foc_app_level_foc_motor_id;
  } else if (unMotorID == g_foc_app_bidet_motor_id) {
    mapped_motor = g_foc_app_bidet_foc_motor_id;
  } else if (unMotorID < FOC_PHY_MOTOR_COUNT) {
    mapped_motor = unMotorID;
  } else {
    return FOC_MOTOR_ID_INVALID;
  }

  if (FOC_AI_CheckPhysicalMotorId(mapped_motor) != FOC_SUCCESS) {
    return FOC_MOTOR_ID_INVALID;
  }

  *punMotorNum = mapped_motor;
  return FOC_SUCCESS;
}

static FocError FOC_AI_SelectMotor(uint8_t unId)
{
  /* 保存电机号检查结果；成功后才允许切换FOC核心当前电机。 */
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

static FocError FOC_AI_CheckReferenceState(void)
{
  /* 当前已选中电机的FOC上下文，用于判断fault状态并映射错误码。 */
  const FOC_Context_t *ctx = FOC_Core_GetContext();

  if (ctx->state == FOC_STATE_FAULT) {
    return FOC_AI_MapFault(ctx->fault);
  }
  return FOC_SUCCESS;
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

static float FOC_AI_SignedMechSpeed(const FOC_Context_t *ctx, float speed)
{
  return (ctx->direction == FOC_DIR_CCW) ? speed : -speed;
}

static FocError FOC_AI_MakeSignedTarget(uint16_t direction,
                                        float magnitude,
                                        float *target)
{
  /* direction来自应用层unParam1，magnitude是速度rpm/电流A/电压V的无符号幅值。 */
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

static void FOC_TestCase_Service(void);

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

  FOC_MainLoop();

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
  /* FOC核心初始化配置，当前两个物理电机共用同一套默认参数初始化。 */
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
  /* ctx指向选中电机的运行上下文，用于判断当前是否running/fault。 */
  const FOC_Context_t *ctx;
  /* err保存电机选择或底层启动结果映射后的External错误码。 */
  FocError err = FOC_AI_SelectMotor(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  ctx = FOC_Core_GetContext();
  if (ctx->state == FOC_STATE_RUNNING) {
    return FOC_SUCCESS;
  }
  if (ctx->state == FOC_STATE_FAULT) {
    err = FOC_AI_MapResult(FOC_ClearFault());
    if (err != FOC_SUCCESS) {
      return err;
    }
  }

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
  /* err保存电机选择结果，选择失败时不再调用FOC_Stop。 */
  FocError err = FOC_AI_SelectMotor(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

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
  /* err保存电机选择、fault状态检查和底层设置结果。 */
  FocError err = FOC_AI_SelectMotor(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_CheckReferenceState();
  if (err != FOC_SUCCESS) {
    return err;
  }

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
  /* target是由方向参数和幅值参数合成后的带符号目标值。 */
  float target;
  /* err保存电机选择、方向转换和下游set接口返回值。 */
  FocError err;

  /* unParam2当前没有参与实际控制，保留是为了匹配外部接口定义。 */
  (void)unParam2;

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  if ((unMode == FOC_APP_MODE_VQ_RATIO_SPEED) ||
      (unMode == FOC_APP_MODE_SPEED)) {
    err = FOC_AI_MakeSignedTarget(unParam1, (float)unParam4, &target);
    if (err != FOC_SUCCESS) {
      return err;
    }
    return Foc_SetSpeedReference_AI(unId, target);
  }

  if ((unMode == FOC_APP_MODE_VQ_RATIO_CURRENT) ||
      (unMode == FOC_APP_MODE_CURRENT)) {
    err = FOC_AI_MakeSignedTarget(unParam1, (float)unParam5 * 0.001f, &target);
    if (err != FOC_SUCCESS) {
      return err;
    }
    return Foc_SetCurrentReference_AI(unId, 0.0f, target);
  }

  if (unMode == FOC_APP_MODE_VQ) {
    err = FOC_AI_MakeSignedTarget(unParam1, (float)unParam3 * 0.001f, &target);
    if (err != FOC_SUCCESS) {
      return err;
    }
    return Foc_SetVoltageReference_AI(unId, 0.0f, target);
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
  /* err保存电机选择、fault状态检查和速度目标设置结果。 */
  FocError err;

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_CheckReferenceState();
  if (err != FOC_SUCCESS) {
    return err;
  }

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
  /* ctx指向选中电机的实时状态，后续字段都从该上下文拷贝或换算得到。 */
  const FOC_Context_t *ctx;
  /* hall_distance保存FOC当前对齐后的带符号Hall行程。 */
  int64_t hall_distance = 0;
  /* total_hall_counts保存有效Hall更新总次数。 */
  uint64_t total_hall_counts = 0U;
  /* direction_hall_counts保存当前方向连续有效Hall更新次数。 */
  uint64_t direction_hall_counts = 0U;
  /* err保存电机选择结果。 */
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

  if (FOC_Core_ReadHallStats(unId,
                             &hall_distance,
                             &total_hall_counts,
                             &direction_hall_counts) == FOC_OK) {
    pstMotorFullStates->nHallDistance = hall_distance;
    pstMotorFullStates->unTotalHallCounts = total_hall_counts;
    pstMotorFullStates->unCurrentDirectionHallCounts = direction_hall_counts;
  }

  /* 以下字段是对应用层返回的电机快照，unId保持调用者传入的FOC物理电机号。 */
  pstMotorFullStates->unId = unId;
  pstMotorFullStates->unHallState = FOC_AI_HallRawToU8(&ctx->hall_raw);
  pstMotorFullStates->fSpeedMechEstimate =
      FOC_AI_SignedMechSpeed(ctx, ctx->speed_fdb);
  pstMotorFullStates->fSpeedMechEstimateFiltered =
      FOC_AI_SignedMechSpeed(ctx, ctx->speed_filtered);
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
  if (FOC_Core_CopyHallHistory(
          unId,
          pstMotorFullStates->punDeltaTimeUsHall,
          pstMotorFullStates->punHistoryHall,
          pstMotorFullStates->punHallCountsHistory,
          (uint16_t)(sizeof(pstMotorFullStates->punDeltaTimeUsHall) /
                     sizeof(pstMotorFullStates->punDeltaTimeUsHall[0])),
          &pstMotorFullStates->unHeadIndexHall,
          &pstMotorFullStates->unHeadIndexAppHall) != FOC_OK) {
    pstMotorFullStates->unHeadIndexHall = -1;
    pstMotorFullStates->unHeadIndexAppHall = -1;
  }
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
  /* err保存应用层电机ID到FOC物理电机号的映射结果。 */
  FocError err;
  (void)unCarConfigID;
  (void)unSeatID;

  g_foc_last_get_motor_num_app_id = unMotorID;
  g_foc_last_get_motor_num_foc_id = 0xFFU;

  if (punMotorNum == NULL) {
    g_foc_last_get_motor_num_err = FOC_POINTER_NULL;
    return FOC_POINTER_NULL;
  }

  err = FOC_AI_MapAppMotorNum(unMotorID, punMotorNum);
  g_foc_last_get_motor_num_err = (uint16_t)err;
  if (err != FOC_SUCCESS) {
    return err;
  }

  g_foc_last_get_motor_num_foc_id = *punMotorNum;
  return FOC_SUCCESS;
}

 

FocError Foc_SetVoltageReference_AI(uint8_t unId, float fVd, float fVq)
{
  /* err保存电机号合法性检查结果；该接口当前只做参数检查，未下发电压目标。 */
  FocError err = FOC_AI_CheckMotorId(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  if ((FOC_FABS(fVd) > 20.0f) || (FOC_FABS(fVq) > 20.0f)) {
    return FOC_VOLTAGE_SETTING_RANGE_INVALID;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

FocError Foc_SetTorqueReference_AI(uint8_t unId, float fTorque)
{
  /* err保存电机号合法性检查结果；扭矩闭环当前未实现。 */
  FocError err = FOC_AI_CheckMotorId(unId);

  (void)fTorque;

  if (err != FOC_SUCCESS) {
    return err;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

FocError Foc_SetIFReference_AI(uint8_t unId, float fIq, float fSpeed)
{
  /* err保存电机号合法性检查结果；I/F控制当前未实现。 */
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
  /* err保存电机号合法性检查结果；V/F控制当前未实现。 */
  FocError err = FOC_AI_CheckMotorId(unId);

  (void)fSpeed;

  if (err != FOC_SUCCESS) {
    return err;
  }

  if (FOC_FABS(fVq) > 20.0f) {
    return FOC_VOLTAGE_SETTING_RANGE_INVALID;
  }

  return FOC_INPUT_PARAMETER_INVALID;
}

FocError Foc_GetAngleAndSpeed_AI(uint8_t unId, float *pfThetaElec, float *pfSpeed)
{
  /* ctx指向选中电机上下文，用于读取电角度和机械速度反馈。 */
  const FOC_Context_t *ctx;
  /* err保存电机选择结果。 */
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
  *pfSpeed = FOC_AI_SignedMechSpeed(ctx, ctx->speed_fdb);

  return FOC_SUCCESS;
}

FocError Foc_ReadMotorHallStates_AI(uint8_t unId, int16_t *pstHallStatesOffset)
{
  /* pstHallStatesOffset输出FOC累计Hall行程加应用层写入offset后的当前位置。 */
  if (pstHallStatesOffset == NULL) {
    return FOC_POINTER_NULL;
  }

  if (FOC_Core_ReadHallTravel(unId, pstHallStatesOffset) != FOC_OK) {
    return FOC_MOTOR_ID_INVALID;
  }

  return FOC_SUCCESS;
}

FocError Foc_WriteMotorHallStates_AI(uint8_t unId, int16_t *pstHallStatesOffset, int16_t nHallDistanceOffset)
{
  /*
   * nHallDistanceOffset当前按应用层绝对Hall位置使用。
   * FOC核心会计算offset，使后续Read返回值对齐该位置。
   */
  if (pstHallStatesOffset == NULL) {
    return FOC_POINTER_NULL;
  }

  if (FOC_Core_WriteHallTravelOffset(unId,
                                     pstHallStatesOffset,
                                     nHallDistanceOffset) != FOC_OK) {
    return FOC_MOTOR_ID_INVALID;
  }

  return FOC_SUCCESS;
}
FOC_AI_DEBUG_ROOT volatile float gfSpeedTarget = 0.0f;

FOC_AI_DEBUG_ROOT volatile uint8_t gunCtrl = 0U;

static float s_foc_test_case_last_fixed_ref = 0.0f;
static uint8_t s_foc_cradle_active = 0U;
static uint32_t s_foc_cradle_start_us = 0U;
static uint32_t s_foc_cradle_last_service_us = 0U;
static uint32_t s_foc_cradle_last_ref_us = 0U;
static uint32_t s_foc_cradle_hold_start_us = 0U;
static float s_foc_cradle_last_ref_rpm = 0.0f;
static int16_t s_foc_cradle_center_hall = 0;
static int8_t s_foc_cradle_target_side = 1;

#define FOC_CRADLE_SEAT_MOTOR_ID             1U
#define FOC_CRADLE_MIN_AMPLITUDE_HALL        4U
#define FOC_CRADLE_MIN_MAX_RPM               30U
#define FOC_CRADLE_ERROR_READ_HALL           0xE1U
#define FOC_CRADLE_ERROR_SELECT_MOTOR        0xE2U
#define FOC_CRADLE_ERROR_SET_REF             0xE3U
#define FOC_CRADLE_ERROR_ENABLE              0xE4U

static uint8_t FOC_TestCase_GetEffectiveMotorId(uint8_t test_case);
static void FOC_TestCase_ResetCradle(void);
static void FOC_TestCase_StartCradle(uint32_t now_us);
static void FOC_TestCase_ServiceCradle(void);

static uint8_t FOC_TestCase_GetMotorId(void)
{
  return (g_foc_test_motor_id < FOC_PHY_MOTOR_COUNT)
       ? g_foc_test_motor_id
       : 0U;
}

static uint8_t FOC_TestCase_GetEffectiveMotorId(uint8_t test_case)
{
  if (test_case == FOC_TEST_CASE_CRADLE_SEAT) {
    return FOC_CRADLE_SEAT_MOTOR_ID;
  }

  return FOC_TestCase_GetMotorId();
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
  FOC_TestCase_ResetCradle();
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

static int16_t FOC_TestCase_ToI16(float value)
{
  if (value > 32767.0f) {
    return 32767;
  }
  if (value < -32768.0f) {
    return -32768;
  }
  return (int16_t)value;
}

static int16_t FOC_TestCase_ClampI32ToI16(int32_t value)
{
  if (value > 32767) {
    return 32767;
  }
  if (value < -32768) {
    return -32768;
  }
  return (int16_t)value;
}

static uint16_t FOC_TestCase_AbsI32ToU16(int32_t value)
{
  uint32_t abs_value = (value < 0) ? (uint32_t)(-value) : (uint32_t)value;

  return (abs_value > 65535U) ? 65535U : (uint16_t)abs_value;
}

static float FOC_TestCase_SmoothStep(float x)
{
  x = FOC_CLAMP(x, 0.0f, 1.0f);
  return x * x * (3.0f - (2.0f * x));
}

static int8_t FOC_TestCase_CradlePolarity(void)
{
  return (g_foc_cradle_cmd_polarity < 0) ? -1 : 1;
}

static float FOC_TestCase_LimitCradleSlew(float target_rpm, uint32_t now_us)
{
  uint16_t slew_rpm_per_s = g_foc_cradle_slew_rpm_per_s;
  float limited = target_rpm;

  if ((slew_rpm_per_s != 0U) && (s_foc_cradle_last_ref_us != 0U)) {
    uint32_t dt_us = now_us - s_foc_cradle_last_ref_us;
    float step = (float)slew_rpm_per_s * ((float)dt_us * 0.000001f);
    float delta = target_rpm - s_foc_cradle_last_ref_rpm;

    if (delta > step) {
      limited = s_foc_cradle_last_ref_rpm + step;
    } else if (delta < -step) {
      limited = s_foc_cradle_last_ref_rpm - step;
    }
  }

  s_foc_cradle_last_ref_us = now_us;
  s_foc_cradle_last_ref_rpm = limited;
  return limited;
}

static void FOC_TestCase_UpdateCradleDebug(int16_t pos,
                                           int16_t target,
                                           int32_t error,
                                           float speed_ref,
                                           uint32_t now_us)
{
  g_foc_cradle_active = s_foc_cradle_active;
  g_foc_cradle_hold_active = (s_foc_cradle_hold_start_us != 0U) ? 1U : 0U;
  g_foc_cradle_target_side = s_foc_cradle_target_side;
  g_foc_cradle_center_hall = s_foc_cradle_center_hall;
  g_foc_cradle_pos_hall = pos;
  g_foc_cradle_target_hall = target;
  g_foc_cradle_error_hall = FOC_TestCase_ClampI32ToI16(error);
  g_foc_cradle_abs_error_hall = FOC_TestCase_AbsI32ToU16(error);
  g_foc_cradle_speed_ref_rpm = FOC_TestCase_ToI16(speed_ref);
  g_foc_cradle_elapsed_ms =
      (s_foc_cradle_start_us == 0U) ? 0U : ((now_us - s_foc_cradle_start_us) / 1000U);
}

static void FOC_TestCase_ResetCradle(void)
{
  s_foc_cradle_active = 0U;
  s_foc_cradle_start_us = 0U;
  s_foc_cradle_last_service_us = 0U;
  s_foc_cradle_last_ref_us = 0U;
  s_foc_cradle_hold_start_us = 0U;
  s_foc_cradle_last_ref_rpm = 0.0f;
  s_foc_cradle_target_side = 1;

  g_foc_cradle_active = 0U;
  g_foc_cradle_hold_active = 0U;
  g_foc_cradle_target_side = 1;
  g_foc_cradle_speed_ref_rpm = 0;
  g_foc_cradle_elapsed_ms = 0U;
}

static void FOC_TestCase_StartCradle(uint32_t now_us)
{
  int16_t pos = 0;

  if (FOC_Core_ReadHallTravel(FOC_CRADLE_SEAT_MOTOR_ID, &pos) != FOC_OK) {
    g_foc_test_case_last_error = FOC_CRADLE_ERROR_READ_HALL;
    s_foc_cradle_active = 0U;
    g_foc_cradle_active = 0U;
    return;
  }

  s_foc_cradle_center_hall = pos;
  s_foc_cradle_start_us = now_us;
  s_foc_cradle_last_service_us = 0U;
  s_foc_cradle_last_ref_us = 0U;
  s_foc_cradle_hold_start_us = 0U;
  s_foc_cradle_last_ref_rpm = 0.0f;
  s_foc_cradle_target_side = 1;
  s_foc_cradle_active = 1U;

  g_foc_cradle_reset_center = 0U;
  g_foc_cradle_cycle_count = 0U;
  FOC_TestCase_UpdateCradleDebug(pos, pos, 0, 0.0f, now_us);
}

static float FOC_TestCase_CalcCradleSpeed(int32_t error, uint16_t deadband)
{
  uint16_t max_rpm_u16 = g_foc_cradle_max_rpm;
  uint16_t slow_zone = g_foc_cradle_slow_zone_hall;
  uint16_t min_move = g_foc_cradle_min_move_rpm;
  float abs_error = (float)FOC_TestCase_AbsI32ToU16(error);
  float max_rpm;
  float limit;
  float speed_ref;

  if (max_rpm_u16 < FOC_CRADLE_MIN_MAX_RPM) {
    max_rpm_u16 = FOC_CRADLE_MIN_MAX_RPM;
  }
  if (slow_zone == 0U) {
    slow_zone = 1U;
  }

  max_rpm = (float)max_rpm_u16;
  limit = max_rpm;
  if (abs_error < (float)slow_zone) {
    limit = max_rpm * FOC_TestCase_SmoothStep(abs_error / (float)slow_zone);
  }

  if (abs_error <= (float)deadband) {
    return 0.0f;
  }

  if ((min_move != 0U) && (limit < (float)min_move)) {
    limit = (float)min_move;
  }
  if (limit > max_rpm) {
    limit = max_rpm;
  }

  speed_ref = (float)error * (float)g_foc_cradle_kp_rpm_per_hall;
  speed_ref = FOC_CLAMP(speed_ref, -limit, limit);
  if (FOC_TestCase_CradlePolarity() < 0) {
    speed_ref = -speed_ref;
  }

  return speed_ref;
}

static uint8_t FOC_TestCase_CradleHoldOrSwap(uint32_t now_us,
                                             uint16_t abs_error,
                                             uint16_t deadband)
{
  uint32_t hold_us = (uint32_t)g_foc_cradle_hold_ms * 1000U;

  if (abs_error > deadband) {
    s_foc_cradle_hold_start_us = 0U;
    return 0U;
  }

  if (s_foc_cradle_hold_start_us == 0U) {
    s_foc_cradle_hold_start_us = now_us;
  }

  if ((now_us - s_foc_cradle_hold_start_us) < hold_us) {
    return 1U;
  }

  s_foc_cradle_target_side = -s_foc_cradle_target_side;
  s_foc_cradle_hold_start_us = 0U;
  if (g_foc_cradle_cycle_count < 0xFFFFFFFFU) {
    g_foc_cradle_cycle_count++;
  }

  return 0U;
}

static void FOC_TestCase_WriteCradleSpeed(float speed_ref)
{
  if (FOC_AI_SelectMotor(FOC_CRADLE_SEAT_MOTOR_ID) != FOC_SUCCESS) {
    g_foc_test_case_last_error = FOC_CRADLE_ERROR_SELECT_MOTOR;
    return;
  }

  if (FOC_SetSpeedRef(speed_ref) != FOC_OK) {
    g_foc_test_case_last_error = FOC_CRADLE_ERROR_SET_REF;
  }
}

static void FOC_TestCase_ServiceCradle(void)
{
  uint32_t now_us = FOC_HAL_GetTimestampUs();
  uint32_t control_us = (uint32_t)g_foc_cradle_control_ms * 1000U;
  uint16_t amplitude = g_foc_cradle_amplitude_hall;
  uint16_t deadband = g_foc_cradle_deadband_hall;
  int16_t pos = 0;
  int16_t target;
  int32_t target_i32;
  int32_t error;
  uint16_t abs_error;
  float speed_ref = 0.0f;
  uint8_t holding;

  if ((s_foc_cradle_active == 0U) || (g_foc_cradle_reset_center != 0U)) {
    FOC_TestCase_StartCradle(now_us);
  }

  if (control_us == 0U) {
    control_us = 1000U;
  }
  if ((s_foc_cradle_last_service_us != 0U) &&
      ((now_us - s_foc_cradle_last_service_us) < control_us)) {
    return;
  }
  s_foc_cradle_last_service_us = now_us;

  if (FOC_Core_ReadHallTravel(FOC_CRADLE_SEAT_MOTOR_ID, &pos) != FOC_OK) {
    g_foc_test_case_last_error = FOC_CRADLE_ERROR_READ_HALL;
    FOC_TestCase_WriteCradleSpeed(0.0f);
    return;
  }

  if (amplitude < FOC_CRADLE_MIN_AMPLITUDE_HALL) {
    amplitude = FOC_CRADLE_MIN_AMPLITUDE_HALL;
  }
  if (deadband >= amplitude) {
    deadband = (uint16_t)(amplitude - 1U);
  }

  target_i32 = (int32_t)s_foc_cradle_center_hall +
               ((int32_t)s_foc_cradle_target_side * (int32_t)amplitude);
  target = FOC_TestCase_ClampI32ToI16(target_i32);
  error = (int32_t)target - (int32_t)pos;
  abs_error = FOC_TestCase_AbsI32ToU16(error);

  holding = FOC_TestCase_CradleHoldOrSwap(now_us, abs_error, deadband);
  if (holding == 0U) {
    target_i32 = (int32_t)s_foc_cradle_center_hall +
                 ((int32_t)s_foc_cradle_target_side * (int32_t)amplitude);
    target = FOC_TestCase_ClampI32ToI16(target_i32);
    error = (int32_t)target - (int32_t)pos;
    speed_ref = FOC_TestCase_CalcCradleSpeed(error, deadband);
  }

  speed_ref = FOC_TestCase_LimitCradleSlew(speed_ref, now_us);
  FOC_TestCase_WriteCradleSpeed(speed_ref);
  FOC_TestCase_UpdateCradleDebug(pos, target, error, speed_ref, now_us);
}

static void FOC_TestCase_Apply(uint8_t test_case)
{
  uint8_t unId = FOC_TestCase_GetEffectiveMotorId(test_case);

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
    g_foc_bidir_speed_step_enable = 0U;
    g_foc_bidir_speed_slew_enable = 0U;
    g_foc_bidir_speed_slew_rpm_per_s = 0U;
    g_foc_zero_transfer_enable = 1U;
    g_foc_zero_transfer_enter_rpm = 320U;
    g_foc_zero_transfer_exit_rpm = 110U;
    g_foc_zero_transfer_ms = 240U;
    g_foc_zero_relaunch_ms = 60U;
    g_foc_zero_relaunch_edge_max_us = 50000U;
    g_foc_zero_handoff_ms = 180U;
    g_foc_zero_hold_iq_mA = 350;
    g_foc_zero_breakaway_iq_mA = 800;
    g_foc_zero_iq_slew_mA_per_s = 3000U;
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
    g_foc_zero_relaunch_abort_count = 0U;
    g_foc_detail_log_enable = 1U;
    g_foc_detail_log_decim_ms = 2U;
    g_foc_detail_log_trigger_rpm = 500U;
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

  }else if(test_case == FOC_TEST_CASE_CRADLE_SEAT){

    FocError enable_err;
    uint8_t other_id = (FOC_CRADLE_SEAT_MOTOR_ID == 0U) ? 1U : 0U;

    FOC_TestCase_ClearAutoModes();
    g_foc_test_motor_id = FOC_CRADLE_SEAT_MOTOR_ID;
    (void)Foc_DisableFocControl(other_id);

    enable_err = Foc_EnableFocControl(unId);
    if (enable_err != FOC_SUCCESS) {
      g_foc_test_case_last_error = FOC_CRADLE_ERROR_ENABLE;
      return;
    }

    g_foc_dyn_speed_start_on_max_ref = 0U;
    g_foc_dyn_speed_start_on_max_fdb = 0U;
    g_foc_detail_log_enable = 1U;
    g_foc_detail_log_decim_ms = 5U;
    g_foc_detail_log_trigger_rpm = g_foc_cradle_max_rpm;
    g_foc_detail_log_zero_window_enable = 0U;
    g_foc_detail_log_reset = 1U;

    FOC_TestCase_StartCradle(FOC_HAL_GetTimestampUs());
    FOC_TestCase_WriteCradleSpeed(0.0f);

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
  uint8_t motor_id = FOC_TestCase_GetEffectiveMotorId(test_case);

  if((test_case == s_foc_test_case_last_select) &&
     (motor_id == s_foc_test_case_last_motor_id)){
    if (test_case == FOC_TEST_CASE_FIXED_SPEED) {
      float fixed_ref = FOC_TestCase_GetFixedSpeedRef(motor_id);
      if (FOC_FABS(fixed_ref - s_foc_test_case_last_fixed_ref) >= 0.5f) {
        FOC_TestCase_Apply(test_case);
      }
    } else if (test_case == FOC_TEST_CASE_CRADLE_SEAT) {
      FOC_TestCase_ServiceCradle();
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
