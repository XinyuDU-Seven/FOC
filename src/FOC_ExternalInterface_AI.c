
#include "FOC_ExternalInterface.h"

#include <string.h>

#include "foc_api.h"
#include "foc_config.h"
#include "foc_core.h"
#include "foc_hal_if.h"
#include "foc_math.h"
#include "foc_observer.h"

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
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_ai_init_done = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_ai_init_result = FOC_ERR;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_lazy_init_count = 0U;
/* 动态速度测试使用的外部标志，External接口清自动模式时会间接影响相关测试模式。 */
extern volatile uint8_t g_foc_dyn_speed_start_on_max_fdb;
/* 旧测试入口的速度给定，External正式控制前会置为-1以退出旧测试给定。 */
extern volatile float speed_ref;
extern volatile uint8_t g_foc_bidir_zero_soft_enable;
extern volatile uint16_t g_foc_bidir_zero_soft_start_rpm;
extern volatile uint8_t  g_foc_start_log_enable;
extern volatile uint8_t  g_foc_start_log_reset;
extern volatile uint16_t g_foc_speed_ref_ramp_up_rpm_per_s;
extern volatile uint16_t g_foc_speed_ref_ramp_down_rpm_per_s;
extern volatile uint8_t  g_foc_low_speed_iq_slew_enable;
extern volatile uint16_t g_foc_low_speed_iq_slew_max_rpm;
extern volatile uint16_t g_foc_low_speed_iq_slew_up_mA_per_s;
extern volatile uint16_t g_foc_low_speed_iq_slew_down_mA_per_s;
extern volatile uint16_t g_foc_lift_current_limit_base_mA;
extern volatile uint16_t g_foc_lift_current_limit_boost_mA;
extern volatile uint16_t g_foc_lift_start_overload_iq_mA;
extern volatile uint16_t g_foc_speed_start_breakaway_boost_max_iq_mA;
extern volatile uint8_t  g_foc_speed_start_unstuck_enable;
extern volatile uint16_t g_foc_speed_start_unstuck_max_iq_mA;
extern volatile uint8_t  g_foc_if_edge_sync_enable;
extern volatile uint32_t g_foc_if_edge_sync_count;
extern volatile uint8_t  g_foc_if_edge_sync_sector;
extern volatile uint16_t g_foc_if_edge_sync_theta_if_before;
extern volatile int16_t  g_foc_if_edge_sync_diff_mrad;
extern volatile uint8_t  g_foc_if_edge_calib_enable;
extern volatile uint8_t  g_foc_if_edge_calib_reset;
extern volatile uint8_t  g_foc_if_edge_calib_skip_edges;
extern volatile uint8_t  g_foc_if_edge_calib_target_edges;
extern volatile uint8_t  g_foc_if_edge_calib_done;
extern volatile uint16_t g_foc_if_edge_calib_sample_count;
extern volatile uint16_t g_foc_if_edge_calib_skipped_count;
extern volatile int16_t  g_foc_if_edge_calib_start_offset_mrad;
extern volatile int32_t  g_foc_if_edge_calib_sum_diff_mrad;
extern volatile uint32_t g_foc_if_edge_calib_abs_sum_diff_mrad;
extern volatile int16_t  g_foc_if_edge_calib_avg_diff_mrad;
extern volatile uint16_t g_foc_if_edge_calib_abs_avg_diff_mrad;
extern volatile int16_t  g_foc_if_edge_calib_min_diff_mrad;
extern volatile int16_t  g_foc_if_edge_calib_max_diff_mrad;
extern volatile int16_t  g_foc_if_edge_calib_recommended_offset_mrad;

#define FOC_DYN_SPEED_LOG_SIZE 512U
#define FOC_DETAIL_LOG_SIZE    512U
#define FOC_APP_SPEED_CMD_LOG_SIZE 128U
#define FOC_SETHYBRID_SPEED_LOG_SIZE       2500U
#define FOC_SETHYBRID_SPEED_LOG_DECIMATION 10U
#define FOC_DETAIL_LOG_DECIM_DEFAULT_MS       12U
#define FOC_DETAIL_LOG_DECIM_STARTUP_MS       2U
#define FOC_DETAIL_LOG_DECIM_SIGNED_CURVE_MS 12U

#define FOC_TEST_CASE_STOP              0U
#define FOC_TEST_CASE_FIXED_SPEED       1U
#define FOC_TEST_CASE_DYN_SPEED_CW      2U
#define FOC_TEST_CASE_SIGNED_CURVE_1000 3U
#define FOC_TEST_CASE_DYN_SPEED_CCW     4U
#define FOC_TEST_CASE_IQ_START_SWEEP    5U
#define FOC_TEST_CASE_IF_EDGE_OFFSET    6U

#define FOC_IF_OFFSET_TEST_PHASE_IDLE          0U
#define FOC_IF_OFFSET_TEST_PHASE_LOCK_FORWARD  1U
#define FOC_IF_OFFSET_TEST_PHASE_SCAN_FORWARD  2U
#define FOC_IF_OFFSET_TEST_PHASE_LOCK_REVERSE  3U
#define FOC_IF_OFFSET_TEST_PHASE_SCAN_REVERSE  4U
#define FOC_IF_OFFSET_TEST_PHASE_DONE          5U

#define FOC_IF_OFFSET_TEST_RESULT_NONE     0U
#define FOC_IF_OFFSET_TEST_RESULT_SUCCESS  1U
#define FOC_IF_OFFSET_TEST_RESULT_TIMEOUT  3U
#define FOC_IF_OFFSET_TEST_RESULT_FAULT    4U
#define FOC_IF_OFFSET_TEST_RESULT_API_ERR  5U

#define FOC_IF_OFFSET_TEST_LOG_SIZE        16U

#define FOC_IQ_START_OFFSET_SCORE_INVALID 2147483647L

static void FOC_IqStartTest_ResetRuntime(uint8_t motor_id);
static void FOC_IFOffsetTest_ResetRuntime(uint8_t motor_id);

FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_testcase1_start_ramp_up_rpm_per_s = 2000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_testcase1_start_ramp_down_rpm_per_s = 1200U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_testcase1_iq_slew_max_rpm = 600U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_testcase1_iq_slew_up_mA_per_s = 4000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_testcase1_iq_slew_down_mA_per_s = 10000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_testcase1_iq_limit_mA = 10000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_testcase1_unstuck_iq_max_mA = 10000U;

static void FOC_TestCase_ClearFixedStartupLimits(void)
{
  FOC_HAL_EnterCritical();
  g_foc_speed_ref_ramp_up_rpm_per_s = 0U;
  g_foc_speed_ref_ramp_down_rpm_per_s = 0U;
  g_foc_low_speed_iq_slew_enable = 0U;
  g_foc_low_speed_iq_slew_max_rpm = 0U;
  g_foc_low_speed_iq_slew_up_mA_per_s = 0U;
  g_foc_low_speed_iq_slew_down_mA_per_s = 0U;
  FOC_HAL_ExitCritical();
}

static void FOC_AI_ApplySpeedStartupLimits(void)
{
  uint16_t iq_limit_mA = g_foc_testcase1_iq_limit_mA;
  uint16_t unstuck_iq_max_mA = g_foc_testcase1_unstuck_iq_max_mA;
  uint16_t ramp_up_rpm_per_s = g_foc_testcase1_start_ramp_up_rpm_per_s;
  uint16_t ramp_down_rpm_per_s = g_foc_testcase1_start_ramp_down_rpm_per_s;
  uint16_t iq_slew_max_rpm = g_foc_testcase1_iq_slew_max_rpm;
  uint16_t iq_slew_up_mA_per_s = g_foc_testcase1_iq_slew_up_mA_per_s;
  uint16_t iq_slew_down_mA_per_s = g_foc_testcase1_iq_slew_down_mA_per_s;
  uint8_t iq_slew_enable =
      ((iq_slew_max_rpm != 0U) &&
       ((iq_slew_up_mA_per_s != 0U) ||
        (iq_slew_down_mA_per_s != 0U))) ? 1U : 0U;

  FOC_HAL_EnterCritical();
  g_foc_lift_current_limit_base_mA = iq_limit_mA;
  g_foc_lift_current_limit_boost_mA = iq_limit_mA;
  g_foc_lift_start_overload_iq_mA = iq_limit_mA;
  g_foc_speed_start_breakaway_boost_max_iq_mA = iq_limit_mA;
  g_foc_speed_start_unstuck_enable = 1U;
  g_foc_speed_start_unstuck_max_iq_mA = unstuck_iq_max_mA;
  g_foc_speed_ref_ramp_up_rpm_per_s = ramp_up_rpm_per_s;
  g_foc_speed_ref_ramp_down_rpm_per_s = ramp_down_rpm_per_s;
  g_foc_low_speed_iq_slew_max_rpm = iq_slew_max_rpm;
  g_foc_low_speed_iq_slew_up_mA_per_s = iq_slew_up_mA_per_s;
  g_foc_low_speed_iq_slew_down_mA_per_s = iq_slew_down_mA_per_s;
  g_foc_low_speed_iq_slew_enable = iq_slew_enable;
  FOC_HAL_ExitCritical();
}

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
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_start_now = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_armed = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_active = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_stop = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_idx = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_decim_ms =
    FOC_DETAIL_LOG_DECIM_DEFAULT_MS;
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
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_speed_err_rpm[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_speed_pid_iq_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_speed_pid_i_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_speed_iq_max_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_speed_error_boost_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_speed_start_state[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_speed_start_iq_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_iq_slew_active[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_iq_slew_limited_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_iq_ref_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_id_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_iq_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_current_peak_mA[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_vq_mV[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_vbus_mV[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_duty_max[FOC_DETAIL_LOG_SIZE];
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
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_detail_log_app_cmd_seq[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_app_cmd_age_ms[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_detail_log_app_direction[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_detail_log_app_speed_rpm[FOC_DETAIL_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_detail_log_app_target_rpm[FOC_DETAIL_LOG_SIZE];

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_speed_cmd_log_enable = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_speed_cmd_log_reset = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_speed_cmd_log_filter_motor_id = 1U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_app_speed_cmd_log_idx = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_speed_cmd_log_wrapped = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_app_speed_cmd_log_seq_next = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_speed_cmd_last_valid = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_app_speed_cmd_last_seq = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_app_speed_cmd_last_t_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_speed_cmd_last_direction = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_app_speed_cmd_last_speed_rpm = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_app_speed_cmd_last_target_rpm = 0;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_app_speed_cmd_log_seq[FOC_APP_SPEED_CMD_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_app_speed_cmd_log_t_ms[FOC_APP_SPEED_CMD_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_app_speed_cmd_log_cb_count[FOC_APP_SPEED_CMD_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_app_speed_cmd_log_direction[FOC_APP_SPEED_CMD_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_app_speed_cmd_log_speed_rpm[FOC_APP_SPEED_CMD_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_app_speed_cmd_log_target_rpm[FOC_APP_SPEED_CMD_LOG_SIZE];

/*
 * Continuous speed trace enabled by successful sethybrid speed commands.
 * Valid samples are always stored chronologically from oldest to newest.
 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_sethybrid_speed_log_enable = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_sethybrid_speed_log_reset = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_sethybrid_speed_log_active = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_sethybrid_speed_log_motor_id = 0xFFU;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_sethybrid_speed_log_wrapped = 0U;
/* Number of valid chronological samples; capped at FOC_SETHYBRID_SPEED_LOG_SIZE. */
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_sethybrid_speed_log_idx = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_sethybrid_speed_log_sample_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_sethybrid_speed_log_current_target_rpm = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_sethybrid_speed_log_target_rpm[FOC_SETHYBRID_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_sethybrid_speed_log_filtered_rpm[FOC_SETHYBRID_SPEED_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_sethybrid_speed_log_actual_rpm[FOC_SETHYBRID_SPEED_LOG_SIZE];

static uint8_t s_foc_sethybrid_speed_log_decim_count = 0U;
static uint8_t s_foc_sethybrid_speed_log_last_motor_id = 0xFFU;

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
 * 3 +/-1000 sine, 4 -1000..-4000 sine,
 * 5 iq start sweep, 6 Id closed-loop/open-angle offset sweep.
 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_select = FOC_TEST_CASE_STOP;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_applied = FOC_TEST_CASE_STOP;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_last_error = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_test_case_exec_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_motor_id = 1U;
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
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_live_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_use_if = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_motor_id = 1U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_current_cmd_id_ref_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_current_cmd_iq_ref_mA = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_current_cmd_if_speed_rpm = 30U;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_id_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_iq_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_current_cmd_result = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_current_cmd_seq = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_live_active = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_current_cmd_applied_use_if = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_current_cmd_applied_id_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_current_cmd_applied_iq_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_current_cmd_applied_if_speed_rpm = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_current_cmd_state = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_current_cmd_fault = 0U;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_id_ref_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_iq_ref_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_id_fdb_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_iq_fdb_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_current_peak_a = 0.0f;
FOC_AI_DEBUG_ROOT volatile float    g_foc_current_cmd_speed_fdb_rpm = 0.0f;

FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_id_mA = 800;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_speed_rpm = 30U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_lock_angle_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_lock_ms = 1000U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_skip_edges = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_target_edges = 12U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_timeout_ms = 8000U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_disable_on_done = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_apply_result = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_applied = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_active = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_done = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_result = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_phase = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_direction = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_if_offset_test_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_if_offset_test_phase_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_api_result = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_state = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_fault = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_speed_fdb_rpm = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_id_cmd_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_iq_cmd_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_id_ref_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_iq_ref_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_id_mA_fdb = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_iq_mA_fdb = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_current_peak_mA = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_cmd_angle_u16 = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_cmd_angle_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_start_offset_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_calib_done = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_calib_samples = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_calib_skipped = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_calib_avg_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_calib_abs_avg_diff_mrad = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_calib_min_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_calib_max_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_calib_recommended_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_if_offset_test_sync_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_sync_sector = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_sync_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_fwd_valid = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_fwd_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_fwd_samples = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_fwd_avg_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_fwd_abs_avg_diff_mrad = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_fwd_min_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_fwd_max_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_fwd_recommended_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_rev_valid = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_rev_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_rev_samples = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_rev_avg_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_rev_abs_avg_diff_mrad = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_rev_min_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_rev_max_diff_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_rev_recommended_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_recommended_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_dir_delta_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_fwd_angle_u16[FOC_IF_OFFSET_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_fwd_angle_mrad[FOC_IF_OFFSET_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_fwd_hall_raw[FOC_IF_OFFSET_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_fwd_hall_sector[FOC_IF_OFFSET_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_if_offset_test_rev_angle_u16[FOC_IF_OFFSET_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_if_offset_test_rev_angle_mrad[FOC_IF_OFFSET_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_rev_hall_raw[FOC_IF_OFFSET_TEST_LOG_SIZE];
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_if_offset_test_rev_hall_sector[FOC_IF_OFFSET_TEST_LOG_SIZE];

FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_start_mA = 200U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_step_mA = 100U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_max_mA = 2500U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_step_ms = 120U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_max_hold_ms = 300U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_move_rpm = 5U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_if_speed_rpm = 30U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_success_edge_count = 6U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_direction = 2U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_disable_on_done = 1U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_active = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_done = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_result = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_iq_start_test_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_cmd_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_iq_ref_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_iq_mA = 0;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_current_peak_mA = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_speed_fdb_rpm = 0;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_start_sector = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_last_sector = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_edge_count = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_first_edge_cmd_mA = 0;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_iq_start_test_first_edge_ms = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_first_speed_cmd_mA = 0;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_iq_start_test_first_speed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_end_cmd_mA = 0;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_offset_mrad = 0;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_offset_score_valid = 0U;
FOC_AI_DEBUG_ROOT volatile int32_t  g_foc_iq_start_test_offset_score =
    FOC_IQ_START_OFFSET_SCORE_INVALID;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_offset_score_cmd_mA = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_iq_start_test_offset_score_time_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_offset_direction = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_offset_best_reset = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_offset_best_valid = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_offset_best_direction = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_offset_best_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int32_t  g_foc_iq_start_test_offset_best_score =
    FOC_IQ_START_OFFSET_SCORE_INVALID;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_offset_best_cmd_mA = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_iq_start_test_offset_best_time_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_offset_fwd_best_valid = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_offset_fwd_best_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int32_t  g_foc_iq_start_test_offset_fwd_best_score =
    FOC_IQ_START_OFFSET_SCORE_INVALID;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_offset_fwd_best_cmd_mA = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_iq_start_test_offset_fwd_best_time_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_iq_start_test_offset_rev_best_valid = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_iq_start_test_offset_rev_best_mrad = 0;
FOC_AI_DEBUG_ROOT volatile int32_t  g_foc_iq_start_test_offset_rev_best_score =
    FOC_IQ_START_OFFSET_SCORE_INVALID;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_offset_rev_best_cmd_mA = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_iq_start_test_offset_rev_best_time_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_state = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_fault = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_iq_start_test_api_result = 0U;

static uint32_t s_foc_iq_start_test_start_us = 0U;
static uint32_t s_foc_iq_start_test_step_us = 0U;
static uint32_t s_foc_iq_start_test_at_max_us = 0U;
static uint16_t s_foc_iq_start_test_abs_cmd_mA = 0U;
static uint32_t s_foc_if_offset_test_start_us = 0U;
static uint32_t s_foc_if_offset_test_phase_start_us = 0U;
static uint32_t s_foc_if_offset_test_last_sync_count = 0U;
static uint8_t s_foc_if_offset_test_prev_sync_enable = 1U;
static uint8_t s_foc_if_offset_test_prev_calib_enable = 1U;
static uint8_t s_foc_if_offset_test_prev_skip_edges = 2U;
static uint8_t s_foc_if_offset_test_prev_target_edges = 12U;
static uint8_t s_foc_current_cmd_live_prev_enable = 0U;
static uint8_t s_foc_current_cmd_last_motor_id = 0xFFU;
static uint8_t s_foc_current_cmd_last_use_if = 0xFFU;
static int16_t s_foc_current_cmd_last_id_mA = 0;
static int16_t s_foc_current_cmd_last_iq_mA = 0;
static int16_t s_foc_current_cmd_last_if_speed_rpm = 0;

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
static volatile uint8_t s_foc_sethybrid_speed_active[FOC_PHY_MOTOR_COUNT] = {
    0U, 0U
};
static volatile uint16_t s_foc_sethybrid_speed_target_rpm[FOC_PHY_MOTOR_COUNT] = {
    0U, 0U
};
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

  if ((ctx->state == FOC_STATE_FAULT) ||
      (ctx->fault != FOC_FAULT_NONE)) {
    return FOC_AI_MapFault(ctx->fault);
  }
  return FOC_SUCCESS;
}

static uint8_t FOC_AI_GetTestCaseMotorId(void)
{
  return (g_foc_test_motor_id < FOC_PHY_MOTOR_COUNT)
       ? g_foc_test_motor_id
       : 0U;
}

static int16_t FOC_AI_GetHallAngleOffsetMrad(uint8_t motor_id)
{
  if (motor_id >= FOC_PHY_MOTOR_COUNT) {
    motor_id = 0U;
  }
  return FOC_Observer_GetHallAngleOffsetMradByMotor(motor_id);
}

static void FOC_AI_SetHallAngleOffsetMrad(uint8_t motor_id,
                                          int16_t offset_mrad)
{
  if (motor_id >= FOC_PHY_MOTOR_COUNT) {
    motor_id = 0U;
  }
  FOC_Observer_SetHallAngleOffsetMrad(motor_id, offset_mrad);
}

static void FOC_AI_DisarmTestCaseService(void)
{
  g_foc_test_case_select = FOC_TEST_CASE_STOP;
  g_foc_test_case_applied = FOC_TEST_CASE_STOP;
  g_foc_test_case_last_error = 0U;
  s_foc_test_case_last_select = FOC_TEST_CASE_STOP;
  s_foc_test_case_last_motor_id = FOC_AI_GetTestCaseMotorId();
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

static int16_t FOC_AI_LogToI16(float value, float scale)
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

static void FOC_AI_SetHybridSpeedLogClearSamples(void)
{
  uint16_t idx;

  for (idx = 0U; idx < FOC_SETHYBRID_SPEED_LOG_SIZE; idx++) {
    g_foc_sethybrid_speed_log_target_rpm[idx] = 0U;
    g_foc_sethybrid_speed_log_filtered_rpm[idx] = 0;
    g_foc_sethybrid_speed_log_actual_rpm[idx] = 0;
  }

  g_foc_sethybrid_speed_log_idx = 0U;
  g_foc_sethybrid_speed_log_wrapped = 0U;
  g_foc_sethybrid_speed_log_sample_count = 0U;
  s_foc_sethybrid_speed_log_decim_count = 0U;
}

static void FOC_AI_SetHybridSpeedLogAppend(uint16_t target_rpm,
                                            int16_t filtered_rpm,
                                            int16_t actual_rpm)
{
  uint16_t idx = g_foc_sethybrid_speed_log_idx;
  uint16_t move_idx;

  if (idx < FOC_SETHYBRID_SPEED_LOG_SIZE) {
    g_foc_sethybrid_speed_log_target_rpm[idx] = target_rpm;
    g_foc_sethybrid_speed_log_filtered_rpm[idx] = filtered_rpm;
    g_foc_sethybrid_speed_log_actual_rpm[idx] = actual_rpm;
    g_foc_sethybrid_speed_log_idx = (uint16_t)(idx + 1U);
  } else {
    /*
     * Keep the exported arrays directly readable as oldest-to-newest data.
     * Once full, discard the oldest point, shift the remaining history left,
     * and append the latest point at the end.
     */
    for (move_idx = 1U;
         move_idx < FOC_SETHYBRID_SPEED_LOG_SIZE;
         move_idx++) {
      g_foc_sethybrid_speed_log_target_rpm[move_idx - 1U] =
          g_foc_sethybrid_speed_log_target_rpm[move_idx];
      g_foc_sethybrid_speed_log_filtered_rpm[move_idx - 1U] =
          g_foc_sethybrid_speed_log_filtered_rpm[move_idx];
      g_foc_sethybrid_speed_log_actual_rpm[move_idx - 1U] =
          g_foc_sethybrid_speed_log_actual_rpm[move_idx];
    }

    g_foc_sethybrid_speed_log_target_rpm[FOC_SETHYBRID_SPEED_LOG_SIZE - 1U] =
        target_rpm;
    g_foc_sethybrid_speed_log_filtered_rpm[FOC_SETHYBRID_SPEED_LOG_SIZE - 1U] =
        filtered_rpm;
    g_foc_sethybrid_speed_log_actual_rpm[FOC_SETHYBRID_SPEED_LOG_SIZE - 1U] =
        actual_rpm;
    g_foc_sethybrid_speed_log_wrapped = 1U;
  }

  if (g_foc_sethybrid_speed_log_sample_count < 0xFFFFFFFFU) {
    g_foc_sethybrid_speed_log_sample_count++;
  }
}

static void FOC_AI_SetHybridSpeedLogStart(uint8_t motor_id,
                                          uint16_t target_rpm)
{
  if (motor_id >= FOC_PHY_MOTOR_COUNT) {
    return;
  }

  s_foc_sethybrid_speed_target_rpm[motor_id] = target_rpm;
  s_foc_sethybrid_speed_active[motor_id] = 1U;

  /* 0xFF selects the first successful sethybrid speed command. The watch
   * variable can later be changed to 0/1 to inspect the other motor. */
  if (g_foc_sethybrid_speed_log_motor_id >= FOC_PHY_MOTOR_COUNT) {
    g_foc_sethybrid_speed_log_motor_id = motor_id;
  }
}

static void FOC_AI_SetHybridSpeedLogStop(uint8_t motor_id)
{
  if (motor_id >= FOC_PHY_MOTOR_COUNT) {
    return;
  }

  s_foc_sethybrid_speed_active[motor_id] = 0U;
  s_foc_sethybrid_speed_target_rpm[motor_id] = 0U;
}

static void FOC_AI_SetHybridSpeedLogService(void)
{
  const FOC_Context_t *ctx;
  uint8_t motor_id;

  if (g_foc_sethybrid_speed_log_reset != 0U) {
    FOC_AI_SetHybridSpeedLogClearSamples();
    g_foc_sethybrid_speed_log_reset = 0U;
  }

  motor_id = g_foc_sethybrid_speed_log_motor_id;
  if (motor_id >= FOC_PHY_MOTOR_COUNT) {
    g_foc_sethybrid_speed_log_active = 0U;
    g_foc_sethybrid_speed_log_current_target_rpm = 0U;
    s_foc_sethybrid_speed_log_decim_count = 0U;
    s_foc_sethybrid_speed_log_last_motor_id = 0xFFU;
    return;
  }

  if (s_foc_sethybrid_speed_log_last_motor_id != motor_id) {
    FOC_AI_SetHybridSpeedLogClearSamples();
    s_foc_sethybrid_speed_log_last_motor_id = motor_id;
  }

  g_foc_sethybrid_speed_log_active =
      s_foc_sethybrid_speed_active[motor_id];
  g_foc_sethybrid_speed_log_current_target_rpm =
      s_foc_sethybrid_speed_target_rpm[motor_id];

  if ((g_foc_sethybrid_speed_log_enable == 0U) ||
      (g_foc_sethybrid_speed_log_active == 0U)) {
    s_foc_sethybrid_speed_log_decim_count = 0U;
    return;
  }

  s_foc_sethybrid_speed_log_decim_count++;
  if (s_foc_sethybrid_speed_log_decim_count <
      FOC_SETHYBRID_SPEED_LOG_DECIMATION) {
    return;
  }
  s_foc_sethybrid_speed_log_decim_count = 0U;

  ctx = FOC_Core_GetContextByMotor(motor_id);
  FOC_AI_SetHybridSpeedLogAppend(
      g_foc_sethybrid_speed_log_current_target_rpm,
      FOC_AI_LogToI16(ctx->speed_ctrl_fdb, 1.0f),
      FOC_AI_LogToI16(ctx->speed_fdb, 1.0f));
}

static void FOC_AI_SpeedCmdLogResetIfNeeded(void)
{
  if (g_foc_app_speed_cmd_log_reset == 0U) {
    return;
  }

  g_foc_app_speed_cmd_log_idx = 0U;
  g_foc_app_speed_cmd_log_wrapped = 0U;
  g_foc_app_speed_cmd_log_seq_next = 0U;
  g_foc_app_speed_cmd_last_valid = 0U;
  g_foc_app_speed_cmd_last_seq = 0U;
  g_foc_app_speed_cmd_last_t_ms = 0U;
  g_foc_app_speed_cmd_last_direction = 0U;
  g_foc_app_speed_cmd_last_speed_rpm = 0U;
  g_foc_app_speed_cmd_last_target_rpm = 0;
  g_foc_app_speed_cmd_log_reset = 0U;
}

static void FOC_AI_RecordHybridSpeedCommand(uint8_t unId,
                                            uint8_t direction,
                                            uint16_t speed_rpm,
                                            float target_rpm)
{
  uint16_t idx;
  uint32_t seq;
  uint32_t t_ms;
  int16_t target_i16;

  FOC_AI_SpeedCmdLogResetIfNeeded();
  if (unId != g_foc_app_speed_cmd_log_filter_motor_id) {
    return;
  }

  g_foc_test_motor_id_applied = unId;

  seq = g_foc_app_speed_cmd_log_seq_next++;
  t_ms = FOC_HAL_GetTimestampUs() / 1000U;
  target_i16 = FOC_AI_LogToI16(target_rpm, 1.0f);

  g_foc_app_speed_cmd_last_valid = 1U;
  g_foc_app_speed_cmd_last_seq = seq;
  g_foc_app_speed_cmd_last_t_ms = t_ms;
  g_foc_app_speed_cmd_last_direction = direction;
  g_foc_app_speed_cmd_last_speed_rpm = speed_rpm;
  g_foc_app_speed_cmd_last_target_rpm = target_i16;

  if (g_foc_app_speed_cmd_log_enable == 0U) {
    return;
  }

  idx = g_foc_app_speed_cmd_log_idx;
  if (idx >= FOC_APP_SPEED_CMD_LOG_SIZE) {
    idx = 0U;
    g_foc_app_speed_cmd_log_wrapped = 1U;
  }

  g_foc_app_speed_cmd_log_idx = (uint16_t)(idx + 1U);
  if (g_foc_app_speed_cmd_log_idx >= FOC_APP_SPEED_CMD_LOG_SIZE) {
    g_foc_app_speed_cmd_log_idx = 0U;
    g_foc_app_speed_cmd_log_wrapped = 1U;
  }

  g_foc_app_speed_cmd_log_seq[idx] = seq;
  g_foc_app_speed_cmd_log_t_ms[idx] = t_ms;
  g_foc_app_speed_cmd_log_cb_count[idx] = g_foc_ai_callback_count;
  g_foc_app_speed_cmd_log_direction[idx] = direction;
  g_foc_app_speed_cmd_log_speed_rpm[idx] = speed_rpm;
  g_foc_app_speed_cmd_log_target_rpm[idx] = target_i16;
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

static uint8_t FOC_AI_IsHybridDisableCommand(uint8_t unMode,
                                             uint16_t unParam1,
                                             uint16_t unParam2,
                                             uint16_t unParam3,
                                             uint16_t unParam4,
                                             uint16_t unParam5)
{
  return ((unMode == 0U) &&
          (unParam1 == 0U) &&
          (unParam2 == 0U) &&
          (unParam3 == 0U) &&
          (unParam4 == 0U) &&
          (unParam5 == 0U)) ? 1U : 0U;
}

static void FOC_AI_ClearAutoModesInternal(uint8_t clear_speed_startup_limits)
{
  speed_ref = -1.0f;
  FOC_AI_DisarmTestCaseService();
  if (clear_speed_startup_limits != 0U) {
    FOC_TestCase_ClearFixedStartupLimits();
  }
  if (g_foc_if_offset_test_active != 0U) {
    FOC_IFOffsetTest_ResetRuntime(FOC_AI_GetTestCaseMotorId());
  }
  g_foc_dyn_speed_enable = 0U;
  g_foc_dyn_speed_reverse = 0U;
  g_foc_dyn_speed_reset_stats = 0U;
  g_foc_bidir_speed_enable = 0U;
  g_foc_bidir_speed_step_enable = 0U;
  g_foc_bidir_speed_reset_stats = 0U;
}

static void FOC_AI_ClearAutoModes(void)
{
  FOC_AI_ClearAutoModesInternal(1U);
}

static void FOC_AI_ClearAutoModesForSpeedReference(void)
{
  FOC_AI_ClearAutoModesInternal(0U);
}

static int16_t FOC_IqStartTest_ToI16(float value, float scale)
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

static uint16_t FOC_IqStartTest_ToU16(float value, float scale)
{
  float scaled = value * scale;

  if (scaled > 65535.0f) {
    return 65535U;
  }
  if (scaled < 0.0f) {
    return 0U;
  }
  return (uint16_t)scaled;
}

static int16_t FOC_IqStartTest_SignedCmdMilli(uint16_t abs_mA)
{
  if (abs_mA > 32767U) {
    abs_mA = 32767U;
  }
  return (g_foc_iq_start_test_direction == FOC_APP_DIR_REVERSE)
       ? -(int16_t)abs_mA
       :  (int16_t)abs_mA;
}

static uint16_t FOC_IqStartTest_AbsCmdMilli(int16_t cmd_mA)
{
  int32_t value = (int32_t)cmd_mA;

  if (value < 0) {
    value = -value;
  }
  return (value > 65535L) ? 65535U : (uint16_t)value;
}

static int32_t FOC_IqStartTest_MakeOffsetScore(uint16_t cmd_mA,
                                                uint32_t time_ms)
{
  int32_t score = (int32_t)cmd_mA * 1000L;

  if (time_ms > (uint32_t)(FOC_IQ_START_OFFSET_SCORE_INVALID - score)) {
    return FOC_IQ_START_OFFSET_SCORE_INVALID;
  }
  return score + (int32_t)time_ms;
}

static void FOC_IqStartTest_ClearBestOffsetScore(void)
{
  g_foc_iq_start_test_offset_best_reset = 0U;
  g_foc_iq_start_test_offset_best_valid = 0U;
  g_foc_iq_start_test_offset_best_direction = 0U;
  g_foc_iq_start_test_offset_best_mrad = 0;
  g_foc_iq_start_test_offset_best_score =
      FOC_IQ_START_OFFSET_SCORE_INVALID;
  g_foc_iq_start_test_offset_best_cmd_mA = 0U;
  g_foc_iq_start_test_offset_best_time_ms = 0U;
  g_foc_iq_start_test_offset_fwd_best_valid = 0U;
  g_foc_iq_start_test_offset_fwd_best_mrad = 0;
  g_foc_iq_start_test_offset_fwd_best_score =
      FOC_IQ_START_OFFSET_SCORE_INVALID;
  g_foc_iq_start_test_offset_fwd_best_cmd_mA = 0U;
  g_foc_iq_start_test_offset_fwd_best_time_ms = 0U;
  g_foc_iq_start_test_offset_rev_best_valid = 0U;
  g_foc_iq_start_test_offset_rev_best_mrad = 0;
  g_foc_iq_start_test_offset_rev_best_score =
      FOC_IQ_START_OFFSET_SCORE_INVALID;
  g_foc_iq_start_test_offset_rev_best_cmd_mA = 0U;
  g_foc_iq_start_test_offset_rev_best_time_ms = 0U;
}

static void FOC_IqStartTest_RecordOffsetScore(uint16_t cmd_mA,
                                               uint32_t time_ms)
{
  int32_t score = FOC_IqStartTest_MakeOffsetScore(cmd_mA, time_ms);
  uint8_t direction = g_foc_iq_start_test_offset_direction;

  g_foc_iq_start_test_offset_score_valid = 1U;
  g_foc_iq_start_test_offset_score = score;
  g_foc_iq_start_test_offset_score_cmd_mA = cmd_mA;
  g_foc_iq_start_test_offset_score_time_ms = time_ms;

  if ((g_foc_iq_start_test_offset_best_valid == 0U) ||
      (score < g_foc_iq_start_test_offset_best_score)) {
    g_foc_iq_start_test_offset_best_valid = 1U;
    g_foc_iq_start_test_offset_best_direction = direction;
    g_foc_iq_start_test_offset_best_mrad =
        g_foc_iq_start_test_offset_mrad;
    g_foc_iq_start_test_offset_best_score = score;
    g_foc_iq_start_test_offset_best_cmd_mA = cmd_mA;
    g_foc_iq_start_test_offset_best_time_ms = time_ms;
  }

  if (direction == FOC_APP_DIR_FORWARD) {
    if ((g_foc_iq_start_test_offset_fwd_best_valid == 0U) ||
        (score < g_foc_iq_start_test_offset_fwd_best_score)) {
      g_foc_iq_start_test_offset_fwd_best_valid = 1U;
      g_foc_iq_start_test_offset_fwd_best_mrad =
          g_foc_iq_start_test_offset_mrad;
      g_foc_iq_start_test_offset_fwd_best_score = score;
      g_foc_iq_start_test_offset_fwd_best_cmd_mA = cmd_mA;
      g_foc_iq_start_test_offset_fwd_best_time_ms = time_ms;
    }
  } else if (direction == FOC_APP_DIR_REVERSE) {
    if ((g_foc_iq_start_test_offset_rev_best_valid == 0U) ||
        (score < g_foc_iq_start_test_offset_rev_best_score)) {
      g_foc_iq_start_test_offset_rev_best_valid = 1U;
      g_foc_iq_start_test_offset_rev_best_mrad =
          g_foc_iq_start_test_offset_mrad;
      g_foc_iq_start_test_offset_rev_best_score = score;
      g_foc_iq_start_test_offset_rev_best_cmd_mA = cmd_mA;
      g_foc_iq_start_test_offset_rev_best_time_ms = time_ms;
    }
  }
}

static void FOC_IqStartTest_ResetRuntime(uint8_t motor_id)
{
  if (g_foc_iq_start_test_offset_best_reset != 0U) {
    FOC_IqStartTest_ClearBestOffsetScore();
  }

  g_foc_iq_start_test_active = 0U;
  g_foc_iq_start_test_done = 0U;
  g_foc_iq_start_test_result = 0U;
  g_foc_iq_start_test_elapsed_ms = 0U;
  g_foc_iq_start_test_cmd_mA = 0;
  g_foc_iq_start_test_iq_ref_mA = 0;
  g_foc_iq_start_test_iq_mA = 0;
  g_foc_iq_start_test_current_peak_mA = 0U;
  g_foc_iq_start_test_speed_fdb_rpm = 0;
  g_foc_iq_start_test_start_sector = 0U;
  g_foc_iq_start_test_last_sector = 0U;
  g_foc_iq_start_test_edge_count = 0U;
  g_foc_iq_start_test_first_edge_cmd_mA = 0;
  g_foc_iq_start_test_first_edge_ms = 0U;
  g_foc_iq_start_test_first_speed_cmd_mA = 0;
  g_foc_iq_start_test_first_speed_ms = 0U;
  g_foc_iq_start_test_end_cmd_mA = 0;
  g_foc_iq_start_test_offset_mrad =
      FOC_AI_GetHallAngleOffsetMrad(motor_id);
  g_foc_iq_start_test_offset_score_valid = 0U;
  g_foc_iq_start_test_offset_score =
      FOC_IQ_START_OFFSET_SCORE_INVALID;
  g_foc_iq_start_test_offset_score_cmd_mA = 0U;
  g_foc_iq_start_test_offset_score_time_ms = 0U;
  g_foc_iq_start_test_offset_direction = g_foc_iq_start_test_direction;
  g_foc_iq_start_test_state = 0U;
  g_foc_iq_start_test_fault = 0U;
  g_foc_iq_start_test_api_result = 0U;
  s_foc_iq_start_test_start_us = 0U;
  s_foc_iq_start_test_step_us = 0U;
  s_foc_iq_start_test_at_max_us = 0U;
  s_foc_iq_start_test_abs_cmd_mA = 0U;
}

static void FOC_IqStartTest_UpdateMonitor(uint8_t motor_id, uint32_t now_us)
{
  const FOC_Context_t *ctx = FOC_Core_GetContextByMotor(motor_id);
  float signed_speed = FOC_AI_SignedMechSpeed(ctx, ctx->speed_fdb);

  if (s_foc_iq_start_test_start_us != 0U) {
    g_foc_iq_start_test_elapsed_ms =
        (now_us - s_foc_iq_start_test_start_us) / 1000U;
  }
  g_foc_iq_start_test_state = (uint16_t)ctx->state;
  g_foc_iq_start_test_fault = (uint16_t)ctx->fault;
  g_foc_iq_start_test_iq_ref_mA =
      FOC_IqStartTest_ToI16(FOC_AI_SignedIqRef(ctx), 1000.0f);
  g_foc_iq_start_test_iq_mA =
      FOC_IqStartTest_ToI16(ctx->i_dq.q, 1000.0f);
  g_foc_iq_start_test_current_peak_mA =
      FOC_IqStartTest_ToU16(ctx->current_peak, 1000.0f);
  g_foc_iq_start_test_speed_fdb_rpm =
      FOC_IqStartTest_ToI16(signed_speed, 1.0f);
}

static FocError FOC_IqStartTest_Command(uint8_t motor_id, uint16_t abs_mA)
{
  float iq_target = 0.0f;
  float speed_target = 0.0f;
  FocError result;

  result = FOC_AI_MakeSignedTarget(g_foc_iq_start_test_direction,
                                   (float)abs_mA * 0.001f,
                                   &iq_target);
  if (result == FOC_SUCCESS) {
    result = FOC_AI_MakeSignedTarget(g_foc_iq_start_test_direction,
                                     (float)g_foc_iq_start_test_if_speed_rpm,
                                     &speed_target);
  }
  if (result == FOC_SUCCESS) {
    result = Foc_SetIFReference(motor_id, iq_target, speed_target);
  }

  s_foc_iq_start_test_abs_cmd_mA = abs_mA;
  g_foc_iq_start_test_cmd_mA = FOC_IqStartTest_SignedCmdMilli(abs_mA);
  g_foc_iq_start_test_api_result = (uint16_t)result;

  return result;
}

static void FOC_IqStartTest_Finish(uint8_t motor_id, uint8_t result)
{
  FocError stop_result;
  uint8_t keep_current =
      ((g_foc_iq_start_test_disable_on_done == 0U) &&
       ((result == 1U) || (result == 2U))) ? 1U : 0U;

  g_foc_iq_start_test_result = result;
  g_foc_iq_start_test_done = 1U;
  g_foc_iq_start_test_active = 0U;
  g_foc_iq_start_test_end_cmd_mA = g_foc_iq_start_test_cmd_mA;

  if (keep_current != 0U) {
    return;
  }

  if ((g_foc_iq_start_test_disable_on_done != 0U) ||
      (result == 4U) ||
      (result == 5U)) {
    stop_result = Foc_DisableFocControl(motor_id);
  } else {
    stop_result = Foc_SetCurrentReference(motor_id, 0.0f, 0.0f);
  }

  if (stop_result != FOC_SUCCESS) {
    g_foc_iq_start_test_api_result = (uint16_t)stop_result;
  }
}

static void FOC_IqStartTest_Start(uint8_t motor_id)
{
  const FOC_Context_t *ctx;
  FocError result;
  uint16_t start_mA = g_foc_iq_start_test_start_mA;
  uint16_t max_mA = g_foc_iq_start_test_max_mA;
  uint32_t now_us = FOC_HAL_GetTimestampUs();

  FOC_IqStartTest_ResetRuntime(motor_id);

  if (max_mA < start_mA) {
    max_mA = start_mA;
  }

  g_foc_iq_start_test_active = 1U;
  s_foc_iq_start_test_start_us = now_us;
  s_foc_iq_start_test_step_us = now_us;
  s_foc_iq_start_test_at_max_us = (start_mA >= max_mA) ? now_us : 0U;

  result = Foc_EnableFocControl(motor_id);
  if (result == FOC_SUCCESS) {
    result = FOC_IqStartTest_Command(motor_id, start_mA);
  }
  if (result != FOC_SUCCESS) {
    g_foc_iq_start_test_api_result = (uint16_t)result;
    FOC_IqStartTest_Finish(motor_id, 5U);
    return;
  }

  ctx = FOC_Core_GetContextByMotor(motor_id);
  g_foc_iq_start_test_start_sector = ctx->hall_sector.sector;
  g_foc_iq_start_test_last_sector = ctx->hall_sector.sector;
  FOC_IqStartTest_UpdateMonitor(motor_id, now_us);
}

static void FOC_IqStartTest_Service(uint8_t motor_id)
{
  const FOC_Context_t *ctx;
  uint32_t now_us;
  uint32_t step_us;
  uint32_t max_hold_us;
  uint16_t step_mA;
  uint16_t max_mA;
  uint16_t success_edges;
  uint8_t cur_sector;
  float speed_abs;

  if (g_foc_iq_start_test_active == 0U) {
    return;
  }

  now_us = FOC_HAL_GetTimestampUs();
  ctx = FOC_Core_GetContextByMotor(motor_id);
  cur_sector = ctx->hall_sector.sector;
  FOC_IqStartTest_UpdateMonitor(motor_id, now_us);

  if ((ctx->state == FOC_STATE_FAULT) || (ctx->fault != FOC_FAULT_NONE)) {
    FOC_IqStartTest_Finish(motor_id, 4U);
    return;
  }

  if ((g_foc_iq_start_test_last_sector != 0U) &&
      (cur_sector != 0U) &&
      (cur_sector != g_foc_iq_start_test_last_sector)) {
    if (g_foc_iq_start_test_edge_count < 65535U) {
      g_foc_iq_start_test_edge_count++;
    }
    if (g_foc_iq_start_test_first_edge_cmd_mA == 0) {
      g_foc_iq_start_test_first_edge_cmd_mA =
          g_foc_iq_start_test_cmd_mA;
      g_foc_iq_start_test_first_edge_ms =
          g_foc_iq_start_test_elapsed_ms;
      FOC_IqStartTest_RecordOffsetScore(
          FOC_IqStartTest_AbsCmdMilli(g_foc_iq_start_test_cmd_mA),
          g_foc_iq_start_test_elapsed_ms);
    }
    g_foc_iq_start_test_last_sector = cur_sector;
    success_edges = g_foc_iq_start_test_success_edge_count;
    if (success_edges == 0U) {
      success_edges = 1U;
    }
    if (g_foc_iq_start_test_edge_count >= success_edges) {
      FOC_IqStartTest_Finish(motor_id, 1U);
      return;
    }
  }
  if (cur_sector != 0U) {
    g_foc_iq_start_test_last_sector = cur_sector;
  }

  speed_abs = ctx->speed_fdb;
  if (speed_abs < 0.0f) {
    speed_abs = -speed_abs;
  }
  if ((g_foc_iq_start_test_move_rpm != 0U) &&
      (speed_abs >= (float)g_foc_iq_start_test_move_rpm)) {
    if (g_foc_iq_start_test_first_speed_cmd_mA == 0) {
      g_foc_iq_start_test_first_speed_cmd_mA =
          g_foc_iq_start_test_cmd_mA;
      g_foc_iq_start_test_first_speed_ms =
          g_foc_iq_start_test_elapsed_ms;
      if (g_foc_iq_start_test_offset_score_valid == 0U) {
        FOC_IqStartTest_RecordOffsetScore(
            FOC_IqStartTest_AbsCmdMilli(g_foc_iq_start_test_cmd_mA),
            g_foc_iq_start_test_elapsed_ms);
      }
    }
    FOC_IqStartTest_Finish(motor_id, 2U);
    return;
  }

  step_mA = g_foc_iq_start_test_step_mA;
  if (step_mA == 0U) {
    step_mA = 100U;
  }
  max_mA = g_foc_iq_start_test_max_mA;
  if (max_mA < g_foc_iq_start_test_start_mA) {
    max_mA = g_foc_iq_start_test_start_mA;
  }
  step_us = (uint32_t)g_foc_iq_start_test_step_ms * 1000U;
  if (step_us < 10000U) {
    step_us = 10000U;
  }

  if ((now_us - s_foc_iq_start_test_step_us) >= step_us) {
    if (s_foc_iq_start_test_abs_cmd_mA < max_mA) {
      uint32_t next_mA =
          (uint32_t)s_foc_iq_start_test_abs_cmd_mA + (uint32_t)step_mA;
      if (next_mA > (uint32_t)max_mA) {
        next_mA = max_mA;
      }
      s_foc_iq_start_test_step_us = now_us;
      if (FOC_IqStartTest_Command(motor_id, (uint16_t)next_mA) !=
          FOC_SUCCESS) {
        FOC_IqStartTest_Finish(motor_id, 5U);
        return;
      }
      if ((uint16_t)next_mA >= max_mA) {
        s_foc_iq_start_test_at_max_us = now_us;
      }
    } else if (s_foc_iq_start_test_at_max_us == 0U) {
      s_foc_iq_start_test_at_max_us = now_us;
    }
  }

  max_hold_us = (uint32_t)g_foc_iq_start_test_max_hold_ms * 1000U;
  if (max_hold_us == 0U) {
    max_hold_us = step_us;
  }
  if ((s_foc_iq_start_test_at_max_us != 0U) &&
      ((now_us - s_foc_iq_start_test_at_max_us) >= max_hold_us)) {
    FOC_IqStartTest_Finish(motor_id, 3U);
  }
}

static int16_t FOC_IFOffsetTest_ClampI32ToI16(int32_t value)
{
  if (value > 32767L) {
    return 32767;
  }
  if (value < -32768L) {
    return -32768;
  }
  return (int16_t)value;
}

static uint16_t FOC_IFOffsetTest_TargetEdges(void)
{
  uint16_t target_edges = g_foc_if_offset_test_target_edges;

  if (target_edges == 0U) {
    target_edges = 12U;
  }
  if (target_edges > FOC_IF_OFFSET_TEST_LOG_SIZE) {
    target_edges = FOC_IF_OFFSET_TEST_LOG_SIZE;
  }
  return target_edges;
}

static uint32_t FOC_IFOffsetTest_TimeoutUs(void)
{
  uint32_t timeout_ms = g_foc_if_offset_test_timeout_ms;

  if (timeout_ms == 0U) {
    timeout_ms = 8000U;
  }
  return timeout_ms * 1000U;
}

static uint32_t FOC_IFOffsetTest_LockUs(void)
{
  uint32_t lock_ms = g_foc_if_offset_test_lock_ms;

  if (lock_ms == 0U) {
    lock_ms = 1000U;
  }
  return lock_ms * 1000U;
}

static uint8_t FOC_IFOffsetTest_PhaseTimedOut(uint32_t now_us)
{
  if (s_foc_if_offset_test_phase_start_us == 0U) {
    return 0U;
  }
  return ((now_us - s_foc_if_offset_test_phase_start_us) >=
          FOC_IFOffsetTest_TimeoutUs()) ? 1U : 0U;
}

static float FOC_IFOffsetTest_LockAngleRad(void)
{
  return FOC_NormalizeAngle((float)g_foc_if_offset_test_lock_angle_mrad *
                            0.001f);
}

static uint16_t FOC_IFOffsetTest_AngleRadToU16(float angle)
{
  angle = FOC_NormalizeAngle(angle);
  return (uint16_t)(angle * (65535.0f / FOC_2PI));
}

static int16_t FOC_IFOffsetTest_AngleU16ToMrad(uint16_t angle_u16)
{
  int32_t angle_mrad =
      (int32_t)(((uint32_t)angle_u16 * 6283U) / 65535U);

  return FOC_IFOffsetTest_ClampI32ToI16(angle_mrad);
}

static void FOC_IFOffsetTest_ClearAngleLog(void)
{
  uint16_t i;

  for (i = 0U; i < FOC_IF_OFFSET_TEST_LOG_SIZE; i++) {
    g_foc_if_offset_test_fwd_angle_u16[i] = 0U;
    g_foc_if_offset_test_fwd_angle_mrad[i] = 0;
    g_foc_if_offset_test_fwd_hall_raw[i] = 0U;
    g_foc_if_offset_test_fwd_hall_sector[i] = 0U;
    g_foc_if_offset_test_rev_angle_u16[i] = 0U;
    g_foc_if_offset_test_rev_angle_mrad[i] = 0;
    g_foc_if_offset_test_rev_hall_raw[i] = 0U;
    g_foc_if_offset_test_rev_hall_sector[i] = 0U;
  }
}

static void FOC_IFOffsetTest_UpdateCombinedResult(void)
{
  int32_t sum = 0;
  uint8_t count = 0U;

  if (g_foc_if_offset_test_fwd_valid != 0U) {
    sum += g_foc_if_offset_test_fwd_recommended_mrad;
    count++;
  }
  if (g_foc_if_offset_test_rev_valid != 0U) {
    sum += g_foc_if_offset_test_rev_recommended_mrad;
    count++;
  }

  if (count != 0U) {
    g_foc_if_offset_test_recommended_mrad =
        FOC_IFOffsetTest_ClampI32ToI16(sum / (int32_t)count);
  }

  if ((g_foc_if_offset_test_fwd_valid != 0U) &&
      (g_foc_if_offset_test_rev_valid != 0U)) {
    g_foc_if_offset_test_dir_delta_mrad =
        FOC_IFOffsetTest_ClampI32ToI16(
            (int32_t)g_foc_if_offset_test_fwd_recommended_mrad -
            (int32_t)g_foc_if_offset_test_rev_recommended_mrad);
  }
}

static void FOC_IFOffsetTest_RecordDirectionResult(uint8_t direction)
{
  uint8_t valid = (g_foc_if_edge_calib_sample_count != 0U) ? 1U : 0U;

  if (direction == FOC_APP_DIR_FORWARD) {
    g_foc_if_offset_test_fwd_valid = valid;
    g_foc_if_offset_test_fwd_samples = g_foc_if_edge_calib_sample_count;
    g_foc_if_offset_test_fwd_avg_diff_mrad =
        g_foc_if_edge_calib_avg_diff_mrad;
    g_foc_if_offset_test_fwd_abs_avg_diff_mrad =
        g_foc_if_edge_calib_abs_avg_diff_mrad;
    g_foc_if_offset_test_fwd_min_diff_mrad =
        g_foc_if_edge_calib_min_diff_mrad;
    g_foc_if_offset_test_fwd_max_diff_mrad =
        g_foc_if_edge_calib_max_diff_mrad;
    g_foc_if_offset_test_fwd_recommended_mrad =
        g_foc_if_edge_calib_recommended_offset_mrad;
  } else if (direction == FOC_APP_DIR_REVERSE) {
    g_foc_if_offset_test_rev_valid = valid;
    g_foc_if_offset_test_rev_samples = g_foc_if_edge_calib_sample_count;
    g_foc_if_offset_test_rev_avg_diff_mrad =
        g_foc_if_edge_calib_avg_diff_mrad;
    g_foc_if_offset_test_rev_abs_avg_diff_mrad =
        g_foc_if_edge_calib_abs_avg_diff_mrad;
    g_foc_if_offset_test_rev_min_diff_mrad =
        g_foc_if_edge_calib_min_diff_mrad;
    g_foc_if_offset_test_rev_max_diff_mrad =
        g_foc_if_edge_calib_max_diff_mrad;
    g_foc_if_offset_test_rev_recommended_mrad =
        g_foc_if_edge_calib_recommended_offset_mrad;
  }

  FOC_IFOffsetTest_UpdateCombinedResult();
}

static void FOC_IFOffsetTest_SaveCoreCalibConfig(void)
{
  s_foc_if_offset_test_prev_sync_enable = g_foc_if_edge_sync_enable;
  s_foc_if_offset_test_prev_calib_enable = g_foc_if_edge_calib_enable;
  s_foc_if_offset_test_prev_skip_edges = g_foc_if_edge_calib_skip_edges;
  s_foc_if_offset_test_prev_target_edges = g_foc_if_edge_calib_target_edges;
}

static void FOC_IFOffsetTest_RestoreCoreCalibConfig(void)
{
  g_foc_if_edge_sync_enable = s_foc_if_offset_test_prev_sync_enable;
  g_foc_if_edge_calib_enable = s_foc_if_offset_test_prev_calib_enable;
  g_foc_if_edge_calib_skip_edges = s_foc_if_offset_test_prev_skip_edges;
  g_foc_if_edge_calib_target_edges =
      s_foc_if_offset_test_prev_target_edges;
  g_foc_if_edge_calib_reset = 0U;
}

static void FOC_IFOffsetTest_PrepareEdgeCalib(void)
{
  g_foc_if_edge_sync_enable = 0U;
  g_foc_if_edge_calib_enable = 1U;
  g_foc_if_edge_calib_skip_edges = g_foc_if_offset_test_skip_edges;
  g_foc_if_edge_calib_target_edges =
      (uint8_t)FOC_IFOffsetTest_TargetEdges();
  g_foc_if_edge_calib_reset = 1U;
  s_foc_if_offset_test_last_sync_count = 0U;
}

static void FOC_IFOffsetTest_UpdateMonitor(uint8_t motor_id, uint32_t now_us)
{
  const FOC_Context_t *ctx = FOC_Core_GetContextByMotor(motor_id);
  float signed_speed = FOC_AI_SignedMechSpeed(ctx, ctx->speed_fdb);
  float cmd_angle = FOC_Core_GetOpenAngleCommand();

  if (s_foc_if_offset_test_start_us != 0U) {
    g_foc_if_offset_test_elapsed_ms =
        (now_us - s_foc_if_offset_test_start_us) / 1000U;
  }
  if (s_foc_if_offset_test_phase_start_us != 0U) {
    g_foc_if_offset_test_phase_elapsed_ms =
        (now_us - s_foc_if_offset_test_phase_start_us) / 1000U;
  }

  g_foc_if_offset_test_state = (uint16_t)ctx->state;
  g_foc_if_offset_test_fault = (uint16_t)ctx->fault;
  g_foc_if_offset_test_speed_fdb_rpm =
      FOC_IqStartTest_ToI16(signed_speed, 1.0f);
  g_foc_if_offset_test_id_ref_mA =
      FOC_IqStartTest_ToI16(ctx->id_ref, 1000.0f);
  g_foc_if_offset_test_iq_ref_mA =
      FOC_IqStartTest_ToI16(FOC_AI_SignedIqRef(ctx), 1000.0f);
  g_foc_if_offset_test_id_mA_fdb =
      FOC_IqStartTest_ToI16(ctx->i_dq.d, 1000.0f);
  g_foc_if_offset_test_iq_mA_fdb =
      FOC_IqStartTest_ToI16(ctx->i_dq.q, 1000.0f);
  g_foc_if_offset_test_current_peak_mA =
      FOC_IqStartTest_ToU16(ctx->current_peak, 1000.0f);
  g_foc_if_offset_test_cmd_angle_u16 =
      FOC_IFOffsetTest_AngleRadToU16(cmd_angle);
  g_foc_if_offset_test_cmd_angle_mrad =
      FOC_IFOffsetTest_AngleU16ToMrad(
          g_foc_if_offset_test_cmd_angle_u16);

  g_foc_if_offset_test_calib_done = g_foc_if_edge_calib_done;
  g_foc_if_offset_test_calib_samples = g_foc_if_edge_calib_sample_count;
  g_foc_if_offset_test_calib_skipped = g_foc_if_edge_calib_skipped_count;
  g_foc_if_offset_test_calib_avg_diff_mrad =
      g_foc_if_edge_calib_avg_diff_mrad;
  g_foc_if_offset_test_calib_abs_avg_diff_mrad =
      g_foc_if_edge_calib_abs_avg_diff_mrad;
  g_foc_if_offset_test_calib_min_diff_mrad =
      g_foc_if_edge_calib_min_diff_mrad;
  g_foc_if_offset_test_calib_max_diff_mrad =
      g_foc_if_edge_calib_max_diff_mrad;
  g_foc_if_offset_test_calib_recommended_mrad =
      g_foc_if_edge_calib_recommended_offset_mrad;
  g_foc_if_offset_test_sync_count = g_foc_if_edge_sync_count;
  g_foc_if_offset_test_sync_sector = g_foc_if_edge_sync_sector;
  g_foc_if_offset_test_sync_diff_mrad = g_foc_if_edge_sync_diff_mrad;
}

static FocError FOC_IFOffsetTest_CommandLock(uint8_t motor_id,
                                             uint8_t direction)
{
  int16_t id_mA = g_foc_if_offset_test_id_mA;
  float id_a = (float)id_mA * 0.001f;
  float lock_angle = FOC_IFOffsetTest_LockAngleRad();
  FocError result;

  g_foc_if_offset_test_direction = direction;
  g_foc_if_offset_test_id_cmd_mA = id_mA;
  g_foc_if_offset_test_iq_cmd_mA = 0;

  g_foc_if_edge_sync_enable = 0U;
  g_foc_if_edge_calib_enable = 0U;
  g_foc_if_edge_calib_reset = 1U;
  s_foc_if_offset_test_last_sync_count = 0U;

  result = FOC_AI_SelectMotor(motor_id);
  if (result == FOC_SUCCESS) {
    result = FOC_AI_CheckReferenceState();
  }
  if (result == FOC_SUCCESS) {
    result = FOC_AI_MapResult(
        FOC_SetOpenAngleCurrentRefAtAngle(id_a, 0.0f, 0.0f, lock_angle));
  }

  g_foc_if_offset_test_api_result = (uint16_t)result;
  return result;
}

static FocError FOC_IFOffsetTest_CommandScan(uint8_t motor_id,
                                             uint8_t direction)
{
  int16_t id_mA = g_foc_if_offset_test_id_mA;
  uint16_t speed_rpm = g_foc_if_offset_test_speed_rpm;
  float id_a = (float)id_mA * 0.001f;
  float rpm = (float)speed_rpm;
  float start_angle = FOC_IFOffsetTest_LockAngleRad();
  FocError result;

  if (direction == FOC_APP_DIR_REVERSE) {
    rpm = -rpm;
  }

  g_foc_if_offset_test_direction = direction;
  g_foc_if_offset_test_id_cmd_mA = id_mA;
  g_foc_if_offset_test_iq_cmd_mA = 0;

  FOC_IFOffsetTest_PrepareEdgeCalib();

  result = FOC_AI_SelectMotor(motor_id);
  if (result == FOC_SUCCESS) {
    result = FOC_AI_CheckReferenceState();
  }
  if (result == FOC_SUCCESS) {
    result = FOC_AI_MapResult(
        FOC_SetOpenAngleCurrentRefAtAngle(id_a, 0.0f, rpm, start_angle));
  }

  g_foc_if_offset_test_api_result = (uint16_t)result;
  return result;
}

static void FOC_IFOffsetTest_RecordEdge(const FOC_Context_t *ctx)
{
  uint16_t idx;
  uint16_t angle_u16;
  int16_t angle_mrad;
  uint8_t hall_raw = FOC_AI_HallRawToU8(&ctx->hall_raw);
  uint8_t hall_sector = g_foc_if_edge_sync_sector;

  if (g_foc_if_edge_sync_count == s_foc_if_offset_test_last_sync_count) {
    return;
  }
  s_foc_if_offset_test_last_sync_count = g_foc_if_edge_sync_count;
  if (g_foc_if_edge_sync_count == 0U) {
    return;
  }

  angle_u16 = g_foc_if_edge_sync_theta_if_before;
  angle_mrad = FOC_IFOffsetTest_AngleU16ToMrad(angle_u16);

  if (g_foc_if_offset_test_phase == FOC_IF_OFFSET_TEST_PHASE_SCAN_FORWARD) {
    idx = g_foc_if_offset_test_fwd_count;
    if (idx < FOC_IF_OFFSET_TEST_LOG_SIZE) {
      g_foc_if_offset_test_fwd_angle_u16[idx] = angle_u16;
      g_foc_if_offset_test_fwd_angle_mrad[idx] = angle_mrad;
      g_foc_if_offset_test_fwd_hall_raw[idx] = hall_raw;
      g_foc_if_offset_test_fwd_hall_sector[idx] = hall_sector;
      g_foc_if_offset_test_fwd_count = (uint16_t)(idx + 1U);
    }
  } else if (g_foc_if_offset_test_phase ==
             FOC_IF_OFFSET_TEST_PHASE_SCAN_REVERSE) {
    idx = g_foc_if_offset_test_rev_count;
    if (idx < FOC_IF_OFFSET_TEST_LOG_SIZE) {
      g_foc_if_offset_test_rev_angle_u16[idx] = angle_u16;
      g_foc_if_offset_test_rev_angle_mrad[idx] = angle_mrad;
      g_foc_if_offset_test_rev_hall_raw[idx] = hall_raw;
      g_foc_if_offset_test_rev_hall_sector[idx] = hall_sector;
      g_foc_if_offset_test_rev_count = (uint16_t)(idx + 1U);
    }
  }
}

static void FOC_IFOffsetTest_ResetRuntime(uint8_t motor_id)
{
  if (g_foc_if_offset_test_active != 0U) {
    FOC_IFOffsetTest_RestoreCoreCalibConfig();
  }

  g_foc_if_offset_test_applied = 0U;
  g_foc_if_offset_test_active = 0U;
  g_foc_if_offset_test_done = 0U;
  g_foc_if_offset_test_result = FOC_IF_OFFSET_TEST_RESULT_NONE;
  g_foc_if_offset_test_phase = FOC_IF_OFFSET_TEST_PHASE_IDLE;
  g_foc_if_offset_test_direction = 0U;
  g_foc_if_offset_test_elapsed_ms = 0U;
  g_foc_if_offset_test_phase_elapsed_ms = 0U;
  g_foc_if_offset_test_api_result = 0U;
  g_foc_if_offset_test_state = 0U;
  g_foc_if_offset_test_fault = 0U;
  g_foc_if_offset_test_speed_fdb_rpm = 0;
  g_foc_if_offset_test_id_cmd_mA = 0;
  g_foc_if_offset_test_iq_cmd_mA = 0;
  g_foc_if_offset_test_id_ref_mA = 0;
  g_foc_if_offset_test_iq_ref_mA = 0;
  g_foc_if_offset_test_id_mA_fdb = 0;
  g_foc_if_offset_test_iq_mA_fdb = 0;
  g_foc_if_offset_test_current_peak_mA = 0U;
  g_foc_if_offset_test_cmd_angle_u16 = 0U;
  g_foc_if_offset_test_cmd_angle_mrad = 0;
  g_foc_if_offset_test_start_offset_mrad =
      FOC_AI_GetHallAngleOffsetMrad(motor_id);
  g_foc_if_offset_test_calib_done = 0U;
  g_foc_if_offset_test_calib_samples = 0U;
  g_foc_if_offset_test_calib_skipped = 0U;
  g_foc_if_offset_test_calib_avg_diff_mrad = 0;
  g_foc_if_offset_test_calib_abs_avg_diff_mrad = 0U;
  g_foc_if_offset_test_calib_min_diff_mrad = 0;
  g_foc_if_offset_test_calib_max_diff_mrad = 0;
  g_foc_if_offset_test_calib_recommended_mrad =
      g_foc_if_offset_test_start_offset_mrad;
  g_foc_if_offset_test_sync_count = 0U;
  g_foc_if_offset_test_sync_sector = 0U;
  g_foc_if_offset_test_sync_diff_mrad = 0;
  g_foc_if_offset_test_fwd_valid = 0U;
  g_foc_if_offset_test_fwd_count = 0U;
  g_foc_if_offset_test_fwd_samples = 0U;
  g_foc_if_offset_test_fwd_avg_diff_mrad = 0;
  g_foc_if_offset_test_fwd_abs_avg_diff_mrad = 0U;
  g_foc_if_offset_test_fwd_min_diff_mrad = 0;
  g_foc_if_offset_test_fwd_max_diff_mrad = 0;
  g_foc_if_offset_test_fwd_recommended_mrad = 0;
  g_foc_if_offset_test_rev_valid = 0U;
  g_foc_if_offset_test_rev_count = 0U;
  g_foc_if_offset_test_rev_samples = 0U;
  g_foc_if_offset_test_rev_avg_diff_mrad = 0;
  g_foc_if_offset_test_rev_abs_avg_diff_mrad = 0U;
  g_foc_if_offset_test_rev_min_diff_mrad = 0;
  g_foc_if_offset_test_rev_max_diff_mrad = 0;
  g_foc_if_offset_test_rev_recommended_mrad = 0;
  g_foc_if_offset_test_recommended_mrad =
      g_foc_if_offset_test_start_offset_mrad;
  g_foc_if_offset_test_dir_delta_mrad = 0;
  FOC_IFOffsetTest_ClearAngleLog();
  s_foc_if_offset_test_start_us = 0U;
  s_foc_if_offset_test_phase_start_us = 0U;
  s_foc_if_offset_test_last_sync_count = 0U;
}

static void FOC_IFOffsetTest_Finish(uint8_t motor_id, uint8_t result)
{
  FocError stop_result;

  FOC_IFOffsetTest_RestoreCoreCalibConfig();

  g_foc_if_offset_test_result = result;
  g_foc_if_offset_test_done = 1U;
  g_foc_if_offset_test_active = 0U;
  g_foc_if_offset_test_phase = FOC_IF_OFFSET_TEST_PHASE_DONE;

  if ((result == FOC_IF_OFFSET_TEST_RESULT_SUCCESS) &&
      (g_foc_if_offset_test_apply_result != 0U) &&
      ((g_foc_if_offset_test_fwd_valid != 0U) ||
       (g_foc_if_offset_test_rev_valid != 0U))) {
    FOC_AI_SetHallAngleOffsetMrad(
        motor_id,
        g_foc_if_offset_test_recommended_mrad);
    g_foc_if_offset_test_applied = 1U;
  }

  if (g_foc_if_offset_test_disable_on_done != 0U) {
    stop_result = Foc_DisableFocControl(motor_id);
  } else {
    (void)FOC_AI_SelectMotor(motor_id);
    stop_result = FOC_AI_MapResult(
        FOC_SetOpenAngleCurrentRef(0.0f, 0.0f, 0.0f));
  }

  if (stop_result != FOC_SUCCESS) {
    g_foc_if_offset_test_api_result = (uint16_t)stop_result;
  }
}

static void FOC_IFOffsetTest_Start(uint8_t motor_id)
{
  FocError result;
  uint32_t now_us = FOC_HAL_GetTimestampUs();

  FOC_IFOffsetTest_ResetRuntime(motor_id);

  FOC_IFOffsetTest_SaveCoreCalibConfig();

  g_foc_if_offset_test_active = 1U;
  g_foc_if_offset_test_phase = FOC_IF_OFFSET_TEST_PHASE_LOCK_FORWARD;
  g_foc_if_offset_test_start_offset_mrad =
      FOC_AI_GetHallAngleOffsetMrad(motor_id);
  g_foc_if_offset_test_calib_recommended_mrad =
      g_foc_if_offset_test_start_offset_mrad;
  g_foc_if_offset_test_recommended_mrad =
      g_foc_if_offset_test_start_offset_mrad;
  s_foc_if_offset_test_start_us = now_us;
  s_foc_if_offset_test_phase_start_us = now_us;

  result = Foc_EnableFocControl(motor_id);
  if (result == FOC_SUCCESS) {
    result = FOC_IFOffsetTest_CommandLock(motor_id, FOC_APP_DIR_FORWARD);
  }
  if (result != FOC_SUCCESS) {
    g_foc_if_offset_test_api_result = (uint16_t)result;
    FOC_IFOffsetTest_Finish(motor_id,
                            FOC_IF_OFFSET_TEST_RESULT_API_ERR);
    return;
  }

  FOC_IFOffsetTest_UpdateMonitor(motor_id, now_us);
}

static void FOC_IFOffsetTest_Service(uint8_t motor_id)
{
  const FOC_Context_t *ctx;
  uint32_t now_us;
  uint16_t target_edges;
  FocError result;

  if (g_foc_if_offset_test_active == 0U) {
    return;
  }

  now_us = FOC_HAL_GetTimestampUs();
  ctx = FOC_Core_GetContextByMotor(motor_id);
  FOC_IFOffsetTest_UpdateMonitor(motor_id, now_us);

  if ((ctx->state == FOC_STATE_FAULT) || (ctx->fault != FOC_FAULT_NONE)) {
    FOC_IFOffsetTest_Finish(motor_id, FOC_IF_OFFSET_TEST_RESULT_FAULT);
    return;
  }

  target_edges = FOC_IFOffsetTest_TargetEdges();

  if (g_foc_if_offset_test_phase ==
      FOC_IF_OFFSET_TEST_PHASE_LOCK_FORWARD) {
    if ((now_us - s_foc_if_offset_test_phase_start_us) >=
        FOC_IFOffsetTest_LockUs()) {
      g_foc_if_offset_test_phase = FOC_IF_OFFSET_TEST_PHASE_SCAN_FORWARD;
      s_foc_if_offset_test_phase_start_us = now_us;
      result = FOC_IFOffsetTest_CommandScan(motor_id, FOC_APP_DIR_FORWARD);
      if (result != FOC_SUCCESS) {
        FOC_IFOffsetTest_Finish(motor_id,
                                FOC_IF_OFFSET_TEST_RESULT_API_ERR);
      }
    }
    return;
  }

  if (g_foc_if_offset_test_phase ==
      FOC_IF_OFFSET_TEST_PHASE_SCAN_FORWARD) {
    FOC_IFOffsetTest_RecordEdge(ctx);
    if (g_foc_if_offset_test_fwd_count >= target_edges) {
      FOC_IFOffsetTest_RecordDirectionResult(FOC_APP_DIR_FORWARD);
      g_foc_if_offset_test_phase = FOC_IF_OFFSET_TEST_PHASE_LOCK_REVERSE;
      s_foc_if_offset_test_phase_start_us = now_us;
      result = FOC_IFOffsetTest_CommandLock(motor_id, FOC_APP_DIR_REVERSE);
      if (result != FOC_SUCCESS) {
        FOC_IFOffsetTest_Finish(motor_id,
                                FOC_IF_OFFSET_TEST_RESULT_API_ERR);
      }
    }
    if (FOC_IFOffsetTest_PhaseTimedOut(now_us) != 0U) {
      FOC_IFOffsetTest_Finish(motor_id, FOC_IF_OFFSET_TEST_RESULT_TIMEOUT);
    }
    return;
  }

  if (g_foc_if_offset_test_phase ==
      FOC_IF_OFFSET_TEST_PHASE_LOCK_REVERSE) {
    if ((now_us - s_foc_if_offset_test_phase_start_us) >=
        FOC_IFOffsetTest_LockUs()) {
      g_foc_if_offset_test_phase = FOC_IF_OFFSET_TEST_PHASE_SCAN_REVERSE;
      s_foc_if_offset_test_phase_start_us = now_us;
      result = FOC_IFOffsetTest_CommandScan(motor_id, FOC_APP_DIR_REVERSE);
      if (result != FOC_SUCCESS) {
        FOC_IFOffsetTest_Finish(motor_id,
                                FOC_IF_OFFSET_TEST_RESULT_API_ERR);
      }
    }
    return;
  }

  if (g_foc_if_offset_test_phase ==
      FOC_IF_OFFSET_TEST_PHASE_SCAN_REVERSE) {
    FOC_IFOffsetTest_RecordEdge(ctx);
    if (g_foc_if_offset_test_rev_count >= target_edges) {
      FOC_IFOffsetTest_RecordDirectionResult(FOC_APP_DIR_REVERSE);
      FOC_IFOffsetTest_Finish(motor_id,
                              FOC_IF_OFFSET_TEST_RESULT_SUCCESS);
      return;
    }
    if (FOC_IFOffsetTest_PhaseTimedOut(now_us) != 0U) {
      FOC_IFOffsetTest_Finish(motor_id, FOC_IF_OFFSET_TEST_RESULT_TIMEOUT);
      return;
    }
    return;
  }

  if ((s_foc_if_offset_test_phase_start_us != 0U) &&
      ((now_us - s_foc_if_offset_test_phase_start_us) >=
       FOC_IFOffsetTest_TimeoutUs())) {
    FOC_IFOffsetTest_Finish(motor_id, FOC_IF_OFFSET_TEST_RESULT_TIMEOUT);
  }
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

static uint8_t FOC_CurrentCmd_GetMotorId(void)
{
  return (g_foc_current_cmd_motor_id < FOC_PHY_MOTOR_COUNT)
       ? g_foc_current_cmd_motor_id
       : 0U;
}

static float FOC_CurrentCmd_MilliToAmp(int16_t current_mA)
{
  return (float)current_mA * 0.001f;
}

static int16_t FOC_CurrentCmd_SignedIfSpeed(int16_t iq_mA)
{
  uint16_t speed_abs = g_foc_current_cmd_if_speed_rpm;

  if (speed_abs > 32767U) {
    speed_abs = 32767U;
  }

  if (iq_mA < 0) {
    return -(int16_t)speed_abs;
  }
  if (iq_mA > 0) {
    return (int16_t)speed_abs;
  }
  return 0;
}

static void FOC_CurrentCmd_ClearLiveCache(void)
{
  s_foc_current_cmd_live_prev_enable = 0U;
  s_foc_current_cmd_last_motor_id = 0xFFU;
  s_foc_current_cmd_last_use_if = 0xFFU;
  s_foc_current_cmd_last_id_mA = 0;
  s_foc_current_cmd_last_iq_mA = 0;
  s_foc_current_cmd_last_if_speed_rpm = 0;
  g_foc_current_cmd_live_active = 0U;
  g_foc_current_cmd_applied_use_if = 0U;
  g_foc_current_cmd_applied_id_mA = 0;
  g_foc_current_cmd_applied_iq_mA = 0;
  g_foc_current_cmd_applied_if_speed_rpm = 0;
}

static void FOC_CurrentCmd_ClearOtherModes(uint8_t motor_id)
{
  g_foc_test_case_select = FOC_TEST_CASE_STOP;
  g_foc_test_case_applied = FOC_TEST_CASE_STOP;
  g_foc_test_motor_id_applied = motor_id;
  s_foc_test_case_last_select = FOC_TEST_CASE_STOP;
  s_foc_test_case_last_motor_id = motor_id;
  g_foc_speed_api_test_enable = 0U;
  g_foc_ext_api_test_enable = 0U;
  FOC_AI_ClearAutoModes();
}

static FocError FOC_CurrentCmd_EnableIfNeeded(uint8_t motor_id)
{
  const FOC_Context_t *ctx = FOC_Core_GetContextByMotor(motor_id);

  if (ctx->state == FOC_STATE_FAULT) {
    return FOC_AI_MapFault(ctx->fault);
  }

  if ((g_foc_current_cmd_auto_enable != 0U) &&
      (ctx->state != FOC_STATE_RUNNING)) {
    return Foc_EnableFocControl(motor_id);
  }

  return FOC_SUCCESS;
}

static FocError FOC_CurrentCmd_ApplyLive(uint8_t motor_id)
{
  FocError result;
  int16_t id_mA = g_foc_current_cmd_id_ref_mA;
  int16_t iq_mA = g_foc_current_cmd_iq_ref_mA;
  int16_t if_speed_rpm = FOC_CurrentCmd_SignedIfSpeed(iq_mA);
  uint8_t use_if =
      ((g_foc_current_cmd_use_if != 0U) && (iq_mA != 0)) ? 1U : 0U;
  float id_a = FOC_CurrentCmd_MilliToAmp(id_mA);
  float iq_a = FOC_CurrentCmd_MilliToAmp(iq_mA);

  g_foc_current_cmd_id_a = id_a;
  g_foc_current_cmd_iq_a = iq_a;

  FOC_CurrentCmd_ClearOtherModes(motor_id);

  result = FOC_CurrentCmd_EnableIfNeeded(motor_id);
  if (result == FOC_SUCCESS) {
    if (use_if != 0U) {
      result = Foc_SetIFReference(motor_id, iq_a, (float)if_speed_rpm);
    } else {
      result = Foc_SetCurrentReference(motor_id, id_a, iq_a);
    }
  }

  g_foc_current_cmd_live_active = (result == FOC_SUCCESS) ? 1U : 0U;
  g_foc_current_cmd_applied_use_if = use_if;
  g_foc_current_cmd_applied_id_mA = id_mA;
  g_foc_current_cmd_applied_iq_mA = iq_mA;
  g_foc_current_cmd_applied_if_speed_rpm = use_if ? if_speed_rpm : 0;

  s_foc_current_cmd_live_prev_enable = 1U;
  s_foc_current_cmd_last_motor_id = motor_id;
  s_foc_current_cmd_last_use_if = g_foc_current_cmd_use_if;
  s_foc_current_cmd_last_id_mA = id_mA;
  s_foc_current_cmd_last_iq_mA = iq_mA;
  s_foc_current_cmd_last_if_speed_rpm = if_speed_rpm;

  return result;
}

static uint8_t FOC_CurrentCmd_LiveNeedsApply(uint8_t motor_id)
{
  int16_t if_speed_rpm = FOC_CurrentCmd_SignedIfSpeed(
      g_foc_current_cmd_iq_ref_mA);

  if (s_foc_current_cmd_live_prev_enable == 0U) {
    return 1U;
  }
  if (motor_id != s_foc_current_cmd_last_motor_id) {
    return 1U;
  }
  if (g_foc_current_cmd_use_if != s_foc_current_cmd_last_use_if) {
    return 1U;
  }
  if (g_foc_current_cmd_id_ref_mA != s_foc_current_cmd_last_id_mA) {
    return 1U;
  }
  if (g_foc_current_cmd_iq_ref_mA != s_foc_current_cmd_last_iq_mA) {
    return 1U;
  }
  if (if_speed_rpm != s_foc_current_cmd_last_if_speed_rpm) {
    return 1U;
  }

  return 0U;
}

static void FOC_CurrentCmd_UpdateMonitor(void)
{
  const FOC_Context_t *ctx;
  uint8_t motor_id = FOC_CurrentCmd_GetMotorId();

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
  uint8_t motor_id = FOC_CurrentCmd_GetMotorId();
  uint8_t has_cmd = 0U;

  if (g_foc_current_cmd_live_enable != 0U) {
    if ((g_foc_current_cmd_apply != 0U) ||
        (FOC_CurrentCmd_LiveNeedsApply(motor_id) != 0U)) {
      g_foc_current_cmd_apply = 0U;
      result = FOC_CurrentCmd_ApplyLive(motor_id);
      has_cmd = 1U;
    }
  } else if (s_foc_current_cmd_live_prev_enable != 0U) {
    result = Foc_SetCurrentReference(motor_id, 0.0f, 0.0f);
    has_cmd = 1U;
    FOC_CurrentCmd_ClearLiveCache();
  }

  if (g_foc_current_cmd_apply != 0U) {
    g_foc_current_cmd_apply = 0U;
    has_cmd = 1U;

    FOC_CurrentCmd_ClearOtherModes(motor_id);

    result = FOC_CurrentCmd_EnableIfNeeded(motor_id);

    if (result == FOC_SUCCESS) {
      result = Foc_SetCurrentReference(motor_id,
                                       g_foc_current_cmd_id_a,
                                       g_foc_current_cmd_iq_a);
    }
  }

  if (g_foc_current_cmd_disable != 0U) {
    g_foc_current_cmd_disable = 0U;
    has_cmd = 1U;
    g_foc_current_cmd_live_enable = 0U;
    g_foc_current_cmd_id_a = 0.0f;
    g_foc_current_cmd_iq_a = 0.0f;
    g_foc_current_cmd_id_ref_mA = 0;
    g_foc_current_cmd_iq_ref_mA = 0;
    FOC_CurrentCmd_ClearLiveCache();
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
  FOC_CurrentCmd_Service();

  FOC_MainLoop();
  FOC_AI_SetHybridSpeedLogService();

}

 

/*******************************************************************************************

  函数名称:  Foc_Init

  函数功能:   FOC电流环控制模块初始化，主要是配置霍尔中断回调函数，使用FOC模块提供的一切API接口前必须先被执行

  输入参数： 无

  输出参数： 无

  返回值:    无

 *******************************************************************************************/

static int FOC_AI_InitCore(void)
{
  /* FOC核心初始化配置，当前两个物理电机共用同一套默认参数初始化。 */
  FOC_Config_t config;
  int result;

  memset(&config, 0, sizeof(FOC_Config_t));

  config.motor.pole_pairs = FOC_APP_POLE_PAIRS;
  config.motor.rs = 0.65f;
  config.motor.ls_d = 0.00007f;
  config.motor.ls_q = 0.00007f;
  config.motor.v_bus = 12.0f;
  config.motor.max_speed_rpm = 4000.0f;
  config.motor.max_current_a = 6.0f;

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

  /*
   * Keep proportional damping strong near the target while limiting the
   * acceleration-phase integral energy that caused the 1000 rpm command to
   * overshoot toward 1200 rpm.  Large-error tracking still has the dedicated
   * speed-error boost in foc_core.c.
   */
  config.speed_pid.kp = 0.0065f;
  config.speed_pid.ki = 0.0012f;
  config.speed_pid.kd = 0.0f;
  config.speed_pid.out_max = config.motor.max_current_a;
  config.speed_pid.out_min = -config.motor.max_current_a;

  result = FOC_Init(&config);
  g_foc_ai_init_result = (int16_t)result;
  g_foc_ai_init_done = (result == FOC_OK) ? 1U : 0U;
  return result;
}

static FocError FOC_AI_EnsureInitialized(void)
{
  if ((g_foc_ai_init_done != 0U) &&
      (g_foc_ai_init_result == FOC_OK)) {
    return FOC_SUCCESS;
  }

  g_foc_ai_lazy_init_count++;
  return FOC_AI_MapResult(FOC_AI_InitCore());
}

void Foc_Init_AI(void)
{
  (void)FOC_AI_InitCore();
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
  FocError err;

  err = FOC_AI_EnsureInitialized();
  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  ctx = FOC_Core_GetContext();
  if ((ctx->state == FOC_STATE_FAULT) ||
      (ctx->fault != FOC_FAULT_NONE)) {
    err = FOC_AI_MapResult(FOC_ClearFault());
    if (err != FOC_SUCCESS) {
      return err;
    }
    ctx = FOC_Core_GetContext();
  }
  if (ctx->state == FOC_STATE_RUNNING) {
    return FOC_SUCCESS;
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
  FocError err;

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  FOC_AI_ClearAutoModes();
  err = FOC_AI_MapResult(FOC_Stop());
  if (err == FOC_SUCCESS) {
    FOC_AI_SetHybridSpeedLogStop(unId);
  }
  return err;
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
  err = FOC_AI_MapResult(FOC_SetCurrentRef(fId, fIq));
  if (err == FOC_SUCCESS) {
    FOC_AI_SetHybridSpeedLogStop(unId);
  }
  return err;
}

static FocError FOC_AI_SetHybridSpeedReference(uint8_t unId, float fSpeed)
{
  FocError err;

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_CheckReferenceState();
  if (err != FOC_SUCCESS) {
    return err;
  }

  FOC_AI_ClearAutoModesForSpeedReference();
  FOC_AI_ApplySpeedStartupLimits();
  return FOC_AI_MapResult(FOC_SetSpeedRef(fSpeed));
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

  err = FOC_AI_EnsureInitialized();
  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  if (FOC_AI_IsHybridDisableCommand(unMode, unParam1, unParam2,
                                    unParam3, unParam4, unParam5) != 0U) {
    FOC_AI_RecordHybridSpeedCommand(unId, FOC_APP_DIR_NONE, 0U, 0.0f);
    FOC_AI_ClearAutoModes();
    err = FOC_AI_MapResult(FOC_Stop());
    if (err == FOC_SUCCESS) {
      FOC_AI_SetHybridSpeedLogStop(unId);
    }
    return err;
  }

  if ((unMode == FOC_APP_MODE_VQ_RATIO_SPEED) ||
      (unMode == FOC_APP_MODE_SPEED)) {
    err = FOC_AI_MakeSignedTarget(unParam1, (float)unParam4, &target);
    if (err != FOC_SUCCESS) {
      return err;
    }
    FOC_AI_RecordHybridSpeedCommand(unId, (uint8_t)unParam1,
                                    unParam4, target);
    err = FOC_AI_SetHybridSpeedReference(unId, target);
    if (err == FOC_SUCCESS) {
      FOC_AI_SetHybridSpeedLogStart(unId, unParam4);
    }
    return err;
  }

  if ((unMode == FOC_APP_MODE_VQ_RATIO_CURRENT) ||
      (unMode == FOC_APP_MODE_CURRENT)) {
    err = FOC_AI_MakeSignedTarget(unParam1, (float)unParam5 * 0.001f, &target);
    if (err != FOC_SUCCESS) {
      return err;
    }
    err = Foc_SetCurrentReference_AI(unId, 0.0f, target);
    return err;
  }

  if (unMode == FOC_APP_MODE_VQ) {
    err = FOC_AI_MakeSignedTarget(unParam1, (float)unParam3 * 0.001f, &target);
    if (err != FOC_SUCCESS) {
      return err;
    }
    err = Foc_SetVoltageReference_AI(unId, 0.0f, target);
    if (err == FOC_SUCCESS) {
      FOC_AI_SetHybridSpeedLogStop(unId);
    }
    return err;
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
  err = FOC_AI_MapResult(FOC_SetSpeedRef(fSpeed));
  if (err == FOC_SUCCESS) {
    FOC_AI_SetHybridSpeedLogStop(unId);
  }
  return err;
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
  FocError err = FOC_AI_CheckMotorId(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_SelectMotor(unId);
  if (err != FOC_SUCCESS) {
    return err;
  }

  err = FOC_AI_MapResult(FOC_SetIFRef(fIq, fSpeed));
  if (err == FOC_SUCCESS) {
    FOC_AI_SetHybridSpeedLogStop(unId);
  }
  return err;
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

static uint8_t FOC_TestCase_GetMotorId(void)
{
  return FOC_AI_GetTestCaseMotorId();
}

static void FOC_TestCase_ClearAutoModes(void)
{
  uint8_t motor_id = FOC_TestCase_GetMotorId();

  speed_ref = -1.0f;
  FOC_TestCase_ClearFixedStartupLimits();
  FOC_IqStartTest_ResetRuntime(motor_id);
  FOC_IFOffsetTest_ResetRuntime(motor_id);
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

static void FOC_TestCase_PrepareFixedSpeedDetailLog(void)
{
  g_foc_start_log_enable = 0U;
  g_foc_start_log_reset = 1U;

  g_foc_detail_log_enable = 1U;
  g_foc_detail_log_decim_ms = FOC_DETAIL_LOG_DECIM_STARTUP_MS;
  g_foc_detail_log_zero_window_enable = 0U;
  g_foc_detail_log_zero_post_ms = 0U;
  g_foc_detail_log_zero_event_idx = 0xFFFFU;
  g_foc_detail_log_zero_event_count = 0U;
  g_foc_detail_log_zero_window_done = 0U;
  g_foc_detail_log_start_now = 0U;
  g_foc_detail_log_reset = 1U;
}

static void FOC_TestCase_KeepSignedCurveDetailLog(void)
{
  g_foc_detail_log_enable = 1U;
  if (g_foc_detail_log_decim_ms < FOC_DETAIL_LOG_DECIM_SIGNED_CURVE_MS) {
    g_foc_detail_log_decim_ms = FOC_DETAIL_LOG_DECIM_SIGNED_CURVE_MS;
    if ((g_foc_detail_log_active != 0U) || (g_foc_detail_log_idx != 0U)) {
      g_foc_detail_log_reset = 1U;
    }
  }
  g_foc_detail_log_trigger_rpm = g_foc_bidir_speed_max_rpm;
  g_foc_detail_log_zero_window_enable = 0U;
  g_foc_detail_log_zero_post_ms = 0U;
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
    FOC_TestCase_PrepareFixedSpeedDetailLog();

    Foc_EnableFocControl(unId);

    if ((FOC_AI_SelectMotor(unId) == FOC_SUCCESS) &&
        (FOC_AI_CheckReferenceState() == FOC_SUCCESS)) {
      FOC_AI_ApplySpeedStartupLimits();
      (void)FOC_SetSpeedRef(fixed_ref);
    }
    g_foc_detail_log_start_now = 1U;
    s_foc_test_case_last_fixed_ref = fixed_ref;

  }else if(test_case == FOC_TEST_CASE_IQ_START_SWEEP){

    FOC_TestCase_ClearAutoModes();
    FOC_TestCase_PrepareFixedSpeedDetailLog();

    g_foc_detail_log_zero_window_enable = 0U;
    g_foc_detail_log_zero_post_ms = 0U;
    FOC_IqStartTest_Start(unId);
    g_foc_detail_log_start_now = 1U;

  }else if(test_case == FOC_TEST_CASE_IF_EDGE_OFFSET){

    FOC_TestCase_ClearAutoModes();
    FOC_TestCase_PrepareFixedSpeedDetailLog();

    g_foc_detail_log_zero_window_enable = 0U;
    g_foc_detail_log_zero_post_ms = 0U;
    FOC_IFOffsetTest_Start(unId);
    g_foc_detail_log_start_now = 1U;

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

  }else if(test_case == FOC_TEST_CASE_SIGNED_CURVE_1000){

    FOC_TestCase_ClearAutoModes();
    Foc_EnableFocControl(unId);

    g_foc_dyn_speed_enable = 0U;
    g_foc_dyn_speed_reverse = 0U;
    g_foc_dyn_speed_reset_stats = 0U;
    g_foc_dyn_speed_start_on_max_ref = 0U;
    g_foc_dyn_speed_start_on_max_fdb = 0U;

    g_foc_bidir_speed_period_ms = 12000U;
    g_foc_bidir_speed_max_rpm = 1000U;
    g_foc_bidir_speed_step_enable = 3U;
    g_foc_bidir_speed_slew_enable = 0U;
    g_foc_bidir_speed_slew_rpm_per_s = 0U;
    g_foc_zero_transfer_enable = 0U;
    g_foc_bidir_zero_soft_enable = 0U;
    g_foc_bidir_zero_cross_enable = 0U;
    g_foc_detail_log_decim_ms = FOC_DETAIL_LOG_DECIM_SIGNED_CURVE_MS;
    FOC_TestCase_KeepSignedCurveDetailLog();
    g_foc_detail_log_zero_event_idx = 0xFFFFU;
    g_foc_detail_log_zero_event_count = 0U;
    g_foc_detail_log_zero_window_done = 0U;
    g_foc_detail_log_reset = 1U;
    g_foc_detail_log_start_now = 0U;

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

  if ((test_case == FOC_TEST_CASE_STOP) &&
      (g_foc_test_case_applied == FOC_TEST_CASE_STOP)) {
    s_foc_test_case_last_select = FOC_TEST_CASE_STOP;
    s_foc_test_case_last_motor_id = motor_id;
    return;
  }

  if((test_case == s_foc_test_case_last_select) &&
     (motor_id == s_foc_test_case_last_motor_id)){
    if (test_case == FOC_TEST_CASE_FIXED_SPEED) {
      float fixed_ref = FOC_TestCase_GetFixedSpeedRef(motor_id);
      if (FOC_FABS(fixed_ref - s_foc_test_case_last_fixed_ref) >= 0.5f) {
        FOC_TestCase_Apply(test_case);
      }
    } else if (test_case == FOC_TEST_CASE_IQ_START_SWEEP) {
      FOC_IqStartTest_Service(motor_id);
    } else if (test_case == FOC_TEST_CASE_IF_EDGE_OFFSET) {
      FOC_IFOffsetTest_Service(motor_id);
    } else if (test_case == FOC_TEST_CASE_SIGNED_CURVE_1000) {
      FOC_TestCase_KeepSignedCurveDetailLog();
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
