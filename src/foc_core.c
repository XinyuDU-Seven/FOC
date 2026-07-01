
/**

 * @file foc_core.c

 * @brief FOC 核心调度实现 — 主控制循环与状态机

 */

 

 #include "foc_core.h"

 #include "foc_transform.h"

 #include "foc_pid.h"

 #include "foc_svpwm.h"

 #include "foc_observer.h"

 #include "foc_protection.h"

 #include "foc_math.h"

 #include "foc_hal_if.h"

 #include "foc_config.h"

 #include <stddef.h>

 #include <string.h>

 

 /* ===================================================================

  *  模块内部状态

  * =================================================================== */

 

 /** FOC 运行上下文实例 */

#define FOC_CORE_MOTOR_COUNT 2U

FOC_Context_t g_foc_ctx[FOC_CORE_MOTOR_COUNT];

 

 /** 模块配置缓存 */

FOC_Config_t  g_foc_config[FOC_CORE_MOTOR_COUNT];

static volatile uint8_t s_foc_core_active_motor = 0U;

#define s_ctx    (g_foc_ctx[s_foc_core_active_motor])
#define s_config (g_foc_config[s_foc_core_active_motor])

#define FOC_CTRL_SOURCE_SPEED   0U
#define FOC_CTRL_SOURCE_CURRENT 1U
static uint8_t s_foc_ctrl_source_store[FOC_CORE_MOTOR_COUNT] = {
    FOC_CTRL_SOURCE_SPEED,
    FOC_CTRL_SOURCE_SPEED
};
#define s_foc_ctrl_source (s_foc_ctrl_source_store[s_foc_core_active_motor])

/** 保护阈值实例 */

FOC_Protection_Threshold_t s_prot_threshold;

/* ===================================================================
 *  FOC debug log buffer
 * =================================================================== */

#ifdef __ICCARM__
#define FOC_DEBUG_ROOT __root
#else
#define FOC_DEBUG_ROOT
#endif

#define FOC_LOG_SIZE        512U
#define FOC_LOG_DECIMATION  1U
#define FOC_TEXT_LOG_SIZE   128U
#define FOC_START_LOG_SIZE  512U
#define FOC_HALL_HISTORY_SIZE 100U

typedef struct {
    uint32_t seq;
    uint32_t t_us;

    int16_t speed_ref_rpm;
    int16_t speed_fdb_rpm;
    int16_t speed_ctrl_fdb_rpm;

    int16_t ia_mA;
    int16_t ib_mA;
    int16_t ic_mA;
    int16_t id_mA;
    int16_t iq_mA;
    int16_t id_ref_mA;
    int16_t iq_ref_mA;
    int16_t speed_error_boost_mA;

    int16_t vd_mV;
    int16_t vq_mV;
    int16_t valpha_mV;
    int16_t vbeta_mV;

    uint16_t vbus_mV;
    uint16_t duty_a;
    uint16_t duty_b;
    uint16_t duty_c;

    uint16_t theta_hall_u16;
    uint16_t theta_pred_u16;
    uint16_t theta_ctrl_u16;

    uint16_t current_peak_mA;
    uint16_t sector_no_change_count;
    uint16_t stall_counter;

    uint8_t hall_raw;
    uint8_t hall_sector;
    uint8_t direction;
    uint8_t state;
    uint8_t fault;
} FOC_LogSample_t;

FOC_DEBUG_ROOT volatile FOC_LogSample_t g_foc_log[FOC_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_foc_log_idx = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_log_fault_idx = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_log_stop = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_log_seq = 0U;

FOC_DEBUG_ROOT volatile uint16_t g_log_idx = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_log_fault_idx = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_log_seq[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint32_t g_log_t_us[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_speed_ref_rpm[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_speed_fdb_rpm[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_speed_ctrl_fdb_rpm[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_ia_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_ib_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_ic_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_id_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_iq_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_id_ref_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_iq_ref_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_speed_error_boost_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_vd_mV[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_log_vq_mV[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_vbus_mV[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_duty_a[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_duty_b[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_duty_c[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_theta_hall[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_theta_pred[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_theta_ctrl[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_current_peak_mA[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_sector_no_change_count[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_stall_counter[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_hall_raw[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_hall_sector[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_direction[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_state[FOC_TEXT_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_log_fault[FOC_TEXT_LOG_SIZE];

FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_enable = 1U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_reset = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_armed = 1U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_active = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_stop = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_filter_motor_id = 0xFFU;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_trigger_motor_id = 0xFFU;
FOC_DEBUG_ROOT volatile uint16_t g_foc_start_log_idx = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_start_log_decim_ms = 10U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_start_log_trigger_ref_rpm = 300U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_start_log_trigger_fdb_max_rpm = 300U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_start_log_trigger_count = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_valid[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_motor_id[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint32_t g_foc_start_log_t_ms[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_ref_rpm[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_ctrl_ref_rpm[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_fdb_rpm[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_ctrl_fdb_rpm[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_err_rpm[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_iq_ref_mA[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_iq_mA[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_foc_start_log_current_peak_mA[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_vq_mV[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_foc_start_log_vbus_mV[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_foc_start_log_duty_max[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_hall_sector[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint16_t g_foc_start_log_sector_no_change_count[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint32_t g_foc_start_log_edge_elapsed_us[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_fault[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile uint8_t  g_foc_start_log_flags[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_lift_extra_mA[FOC_START_LOG_SIZE];
FOC_DEBUG_ROOT volatile int16_t  g_foc_start_log_iq_slew_limited_mA[FOC_START_LOG_SIZE];

/* Prof segment id: 1 state, 2 hall, 3 adc, 4 calc/protect, 5 pwm, 6 log. */
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_enter_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_after_state_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_after_hall_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_after_adc_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_after_calc_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_after_pwm_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_exit_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_period_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_loop_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_max_period_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_max_loop_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_loop_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_max_period_loop = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_max_loop_loop = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_max_seg_loop = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_last_seg_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_prof_max_seg_us = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_prof_last_seg_id = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_prof_max_seg_id = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_late_period_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_late_period_us = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_last_fault_latched = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_last_fault_state = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_last_fault_latch_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_last_fault_seq = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_last_fault_current_peak_mA = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_speed_ref_rpm = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_speed_fdb_rpm = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_last_fault_speed_ctrl_fdb_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_last_fault_iq_ref_mA = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_last_fault_iq_mA = 0;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_sector_no_change_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_stall_counter = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_last_fault_direction = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_last_fault_hall_raw = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_last_fault_hall_sector = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_theta_hall = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_theta_pred = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_theta_ctrl = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_resync_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_hall_resync_period_us = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_resync_prev_sector = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_resync_cur_sector = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_late_recovery_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_late_recovery_period_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_recovery_accept_count = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_recovery_accept_prev_sector = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_recovery_accept_cur_sector = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_post_recovery_duty_slew_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_post_recovery_duty_slew_remaining = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_post_recovery_duty_slew_active = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_recovery_zero_vector_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_recovery_pwm_off_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_recovery_pwm_hold_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_recovery_current_control_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_recovery_zero_vector_remaining = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_recovery_current_wait_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_recovery_release_current_mA = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_recovery_current_wait_active = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_min_time_reject_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_min_time_last_elapsed_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_min_time_last_min_us = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_min_time_prev_sector = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_min_time_cur_sector = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_current_angle_trim_mrad = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_cw_angle_offset_mrad =
    FOC_CW_CONTROL_ANGLE_OFFSET_MRAD;
FOC_DEBUG_ROOT volatile int16_t  g_foc_ccw_angle_offset_mrad =
    FOC_CCW_CONTROL_ANGLE_OFFSET_MRAD;
FOC_DEBUG_ROOT volatile int16_t  g_foc_motor1_cw_angle_offset_mrad = 300;
FOC_DEBUG_ROOT volatile int16_t  g_foc_motor1_ccw_angle_offset_mrad =
    FOC_CCW_CONTROL_ANGLE_OFFSET_MRAD;
FOC_DEBUG_ROOT volatile int16_t  g_foc_control_angle_offset_mrad = 0;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_event_used_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_event_seq = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_event_age_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_poll_count = 0U;
FOC_DEBUG_ROOT volatile int32_t  g_foc_hall_travel_count[FOC_CORE_MOTOR_COUNT] = {0, 0};
FOC_DEBUG_ROOT volatile int32_t  g_foc_hall_travel_offset[FOC_CORE_MOTOR_COUNT] = {0, 0};
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_travel_stall_guard_enable =
    FOC_HALL_TRAVEL_STALL_GUARD_ENABLE;
FOC_DEBUG_ROOT volatile uint16_t g_foc_hall_travel_stall_min_ref_rpm =
    FOC_HALL_TRAVEL_STALL_MIN_REF_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_hall_travel_stall_max_fdb_rpm =
    FOC_HALL_TRAVEL_STALL_MAX_FDB_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_hall_travel_stall_min_err_rpm =
    FOC_HALL_TRAVEL_STALL_MIN_ERR_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_hall_travel_stall_min_iq_mA =
    FOC_HALL_TRAVEL_STALL_MIN_IQ_MA;
FOC_DEBUG_ROOT volatile uint16_t g_foc_hall_travel_stall_count_threshold =
    FOC_HALL_TRAVEL_STALL_COUNT_THRESHOLD;
FOC_DEBUG_ROOT volatile uint16_t g_foc_hall_travel_stall_max_dir_counts =
    FOC_HALL_TRAVEL_STALL_MAX_DIR_COUNTS;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_travel_stall_active[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
FOC_DEBUG_ROOT volatile uint16_t g_foc_hall_travel_stall_counter[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
FOC_DEBUG_ROOT volatile int8_t   g_foc_hall_travel_stall_dir[FOC_CORE_MOTOR_COUNT] = {0, 0};
FOC_DEBUG_ROOT volatile int32_t  g_foc_hall_travel_suppressed_delta[FOC_CORE_MOTOR_COUNT] = {0, 0};
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_travel_freeze_count[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint32_t s_foc_hall_history_delta_us[FOC_CORE_MOTOR_COUNT][FOC_HALL_HISTORY_SIZE];
static uint8_t  s_foc_hall_history_raw[FOC_CORE_MOTOR_COUNT][FOC_HALL_HISTORY_SIZE];
static uint64_t s_foc_hall_history_count[FOC_CORE_MOTOR_COUNT][FOC_HALL_HISTORY_SIZE];
static int16_t  s_foc_hall_history_head[FOC_CORE_MOTOR_COUNT] = {-1, -1};
static uint16_t s_foc_hall_history_valid_count[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint64_t s_foc_hall_total_update_count[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint64_t s_foc_hall_direction_update_count[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static int32_t  s_foc_hall_last_delta[FOC_CORE_MOTOR_COUNT] = {0, 0};
static uint8_t  s_foc_hall_travel_stall_active_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint16_t s_foc_hall_travel_stall_counter_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static int8_t   s_foc_hall_travel_stall_dir_store[FOC_CORE_MOTOR_COUNT] = {0, 0};
static int32_t  s_foc_hall_travel_suppressed_delta_store[FOC_CORE_MOTOR_COUNT] = {0, 0};
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_ctrl_fdb_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_error_boost_mA = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_ref_cmd_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_ref_ctrl_rpm = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_speed_ref_ramp_active = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_ref_ramp_up_rpm_per_s = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_ref_ramp_down_rpm_per_s = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_app_direction_invert_enable = 1U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_low_speed_smooth_enable =
    FOC_LOW_SPEED_SMOOTH_ENABLE;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_low_speed_smooth_active = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_smooth_max_rpm =
    (uint16_t)FOC_LOW_SPEED_SMOOTH_MAX_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_ctrl_alpha_milli =
    (uint16_t)(FOC_LOW_SPEED_CTRL_FILTER_ALPHA * 1000.0f);
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_overspeed_deadband_rpm =
    (uint16_t)FOC_LOW_SPEED_OVERSPEED_DEADBAND_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_ctrl_fdb_no_edge_decay_rpm_per_s =
    FOC_SPEED_CTRL_FDB_NO_EDGE_DECAY_RPM_PER_S;
FOC_DEBUG_ROOT volatile uint32_t g_foc_speed_ctrl_fdb_no_edge_decay_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_ctrl_fdb_max_lead_rpm = 300U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_low_speed_torque_enable =
    FOC_LOW_SPEED_TORQUE_ENABLE;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_low_speed_torque_active = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_torque_max_rpm =
    FOC_LOW_SPEED_TORQUE_MAX_RPM;
FOC_DEBUG_ROOT volatile int16_t  g_foc_low_speed_torque_min_iq_mA =
    FOC_LOW_SPEED_TORQUE_MIN_IQ_MA;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_torque_full_rpm =
    FOC_LOW_SPEED_TORQUE_FULL_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_torque_err_rpm =
    FOC_LOW_SPEED_TORQUE_ERR_RPM;
FOC_DEBUG_ROOT volatile int16_t  g_foc_low_speed_torque_applied_mA = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_low_speed_iq_slew_enable = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_low_speed_iq_slew_active = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_iq_slew_max_rpm = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_iq_slew_up_mA_per_s = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_iq_slew_down_mA_per_s = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_low_speed_iq_slew_limited_mA = 0;
FOC_DEBUG_ROOT volatile uint32_t g_foc_low_speed_iq_slew_count = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_lift_current_limit_enable = 1U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_lift_current_limit_dir = FOC_DIR_CW;
FOC_DEBUG_ROOT volatile uint16_t g_foc_lift_current_limit_base_mA = 5000U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_lift_current_limit_boost_mA = 5000U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_lift_current_limit_max_rpm = 600U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_lift_current_limit_err_rpm = 500U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_lift_current_limit_active = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_lift_current_limit_extra_mA = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_bidir_decel_hold_enable =
    FOC_BIDIR_DECEL_HOLD_ENABLE;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_bidir_decel_hold_active = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_bidir_decel_hold_max_rpm =
    FOC_BIDIR_DECEL_HOLD_MAX_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_bidir_decel_hold_min_iq_mA =
    FOC_BIDIR_DECEL_HOLD_MIN_IQ_MA;
FOC_DEBUG_ROOT volatile uint16_t g_foc_bidir_decel_hold_full_rpm =
    FOC_BIDIR_DECEL_HOLD_FULL_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_bidir_decel_hold_err_rpm =
    FOC_BIDIR_DECEL_HOLD_ERR_RPM;
FOC_DEBUG_ROOT volatile int16_t  g_foc_bidir_decel_hold_raw_err_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_bidir_decel_hold_applied_mA = 0;
FOC_DEBUG_ROOT volatile uint32_t g_foc_bidir_decel_hold_count = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_bidir_zero_soft_enable =
    FOC_BIDIR_ZERO_SOFT_ENABLE;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_bidir_zero_soft_active = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_soft_start_rpm =
    FOC_BIDIR_ZERO_SOFT_START_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_bidir_zero_soft_scale_percent = 100U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_bidir_zero_soft_limited_mA = 0;
FOC_DEBUG_ROOT volatile uint32_t g_foc_bidir_zero_soft_count = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_low_speed_current_ff_enable =
    FOC_LOW_SPEED_CURRENT_FF_ENABLE;
FOC_DEBUG_ROOT volatile uint16_t g_foc_low_speed_current_ff_max_rpm =
    FOC_LOW_SPEED_CURRENT_FF_MAX_RPM;
FOC_DEBUG_ROOT volatile uint16_t g_foc_current_q_rs_ff_gain_milli =
    FOC_CURRENT_Q_RS_FF_GAIN_MILLI;
FOC_DEBUG_ROOT volatile int16_t  g_foc_current_q_ff_mV = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_speed_drop_fault_enable =
    FOC_SPEED_DROP_FAULT_ENABLE;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_ref_min_rpm = 600U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_ref_max_rpm = 1100U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_err_rpm = 350U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_count_limit = 1U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_count = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_drop_fault_ref_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_drop_fault_fdb_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_drop_fault_ctrl_fdb_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_drop_fault_err_last_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_drop_fault_iq_ref_mA = 0;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_min_iq_ref_mA = 800U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_sector_no_change_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_speed_drop_fault_edge_elapsed_us = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_speed_drop_fault_direction = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_speed_drop_fault_bidir_no_edge_skip = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_speed_drop_fault_bidir_no_edge_skip_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_bidir_no_edge_holdoff_cycles = 300U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_drop_fault_bidir_no_edge_holdoff_count = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_speed_fdb_drop_fault_enable = 1U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_min_ref_rpm = 1000U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_min_peak_rpm = 1000U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_delta_rpm = 800U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_ratio_percent = 55U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_count_limit = 10U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_count = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_fdb_drop_fault_ref_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_fdb_drop_fault_fdb_rpm = 0;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_peak_rpm = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_drop_rpm = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_err_rpm = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_speed_fdb_drop_fault_sector_no_change_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_speed_fdb_drop_fault_edge_elapsed_us = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_speed_fdb_drop_fault_direction = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_signed_speed_ref_normalize_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_vbus_mV = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_vbus_mV = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_vbus_brake_limit_mA = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_vbus_brake_active = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_vbus_brake_limited_count = 0U;
FOC_DEBUG_ROOT volatile float    speed_ref = -1.0f;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_dyn_speed_start_on_max_fdb = 1U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_dyn_speed_start_fdb_margin_rpm = 50U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_dyn_core_loop_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_dyn_core_set_ref_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_dyn_core_trigger_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_dyn_core_disable_count = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_dyn_core_seen_ref_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_dyn_core_seen_fdb_rpm = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_dyn_core_state = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_dyn_core_trigger_source = 0U;

#define FOC_DYN_SPEED_LOG_SIZE 512U
#define FOC_DETAIL_LOG_SIZE    512U

extern volatile uint8_t  g_foc_dyn_speed_enable;
extern volatile uint8_t  g_foc_dyn_speed_reverse;
extern volatile uint8_t  g_foc_dyn_speed_start_on_max_ref;
extern volatile int16_t  g_foc_dyn_speed_start_cmd_rpm;
extern volatile uint32_t g_foc_dyn_speed_period_ms;
extern volatile uint16_t g_foc_dyn_speed_min_rpm;
extern volatile uint16_t g_foc_dyn_speed_max_rpm;
extern volatile uint8_t  g_foc_dyn_speed_reset_stats;
extern volatile int16_t  g_foc_dyn_speed_last_ext_ref_rpm;
extern volatile uint32_t g_foc_dyn_speed_elapsed_ms;
extern volatile uint16_t g_foc_dyn_speed_phase_u16;
extern volatile int16_t  g_foc_dyn_speed_ref_rpm;
extern volatile int16_t  g_foc_dyn_speed_fdb_rpm;
extern volatile int16_t  g_foc_dyn_speed_ctrl_fdb_rpm;
extern volatile int16_t  g_foc_dyn_speed_err_rpm;
extern volatile uint16_t g_foc_dyn_speed_abs_err_rpm;
extern volatile uint16_t g_foc_dyn_speed_abs_err_avg_rpm;
extern volatile uint16_t g_foc_dyn_speed_max_abs_err_rpm;
extern volatile int16_t  g_foc_dyn_speed_iq_ref_mA;
extern volatile int16_t  g_foc_dyn_speed_id_mA;
extern volatile int16_t  g_foc_dyn_speed_iq_mA;
extern volatile uint16_t g_foc_dyn_speed_current_peak_mA;
extern volatile uint32_t g_foc_dyn_speed_sample_count;
extern volatile uint16_t g_foc_dyn_log_idx;
extern volatile uint8_t  g_foc_dyn_log_stop;
extern volatile uint16_t g_foc_dyn_log_decim_ms;
extern volatile uint32_t g_foc_dyn_log_t_ms[FOC_DYN_SPEED_LOG_SIZE];
extern volatile int16_t  g_foc_dyn_log_ref_rpm[FOC_DYN_SPEED_LOG_SIZE];
extern volatile int16_t  g_foc_dyn_log_fdb_rpm[FOC_DYN_SPEED_LOG_SIZE];
extern volatile int16_t  g_foc_dyn_log_ctrl_fdb_rpm[FOC_DYN_SPEED_LOG_SIZE];
extern volatile int16_t  g_foc_dyn_log_err_rpm[FOC_DYN_SPEED_LOG_SIZE];
extern volatile int16_t  g_foc_dyn_log_iq_ref_mA[FOC_DYN_SPEED_LOG_SIZE];
extern volatile int16_t  g_foc_dyn_log_iq_mA[FOC_DYN_SPEED_LOG_SIZE];
extern volatile uint16_t g_foc_dyn_log_current_peak_mA[FOC_DYN_SPEED_LOG_SIZE];
extern volatile uint16_t g_foc_dyn_log_fault[FOC_DYN_SPEED_LOG_SIZE];
extern volatile uint8_t  g_foc_detail_log_enable;
extern volatile uint8_t  g_foc_detail_log_reset;
extern volatile uint8_t  g_foc_detail_log_armed;
extern volatile uint8_t  g_foc_detail_log_active;
extern volatile uint8_t  g_foc_detail_log_stop;
extern volatile uint16_t g_foc_detail_log_idx;
extern volatile uint16_t g_foc_detail_log_decim_ms;
extern volatile uint16_t g_foc_detail_log_trigger_rpm;
extern volatile uint32_t g_foc_detail_log_trigger_count;
extern volatile uint8_t  g_foc_detail_log_zero_window_enable;
extern volatile uint16_t g_foc_detail_log_zero_post_ms;
extern volatile uint16_t g_foc_detail_log_zero_event_idx;
extern volatile uint32_t g_foc_detail_log_zero_event_count;
extern volatile uint8_t  g_foc_detail_log_zero_window_done;
extern volatile uint8_t  g_foc_test_motor_id_applied;
extern volatile uint32_t g_foc_detail_log_t_ms[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_raw_ref_rpm[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_ref_rpm[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_fdb_rpm[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_ctrl_fdb_rpm[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_iq_ref_mA[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_id_mA[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_iq_mA[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_current_peak_mA[FOC_DETAIL_LOG_SIZE];
extern volatile uint8_t  g_foc_detail_log_hall_raw[FOC_DETAIL_LOG_SIZE];
extern volatile uint8_t  g_foc_detail_log_hall_sector[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_sector_no_change_count[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_theta_hall[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_theta_pred[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_theta_ctrl[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_duty_a[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_duty_b[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_duty_c[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_zero_soft_scale_percent[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_zero_soft_limited_mA[FOC_DETAIL_LOG_SIZE];
extern volatile uint8_t  g_foc_detail_log_zero_soft_active[FOC_DETAIL_LOG_SIZE];
extern volatile uint8_t  g_foc_detail_log_zero_transfer_state[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_zero_signed_iq_cmd_mA[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_zero_iq_ff_mA[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_zero_ctrl_iq_raw_mA[FOC_DETAIL_LOG_SIZE];
extern volatile uint8_t  g_foc_detail_log_zero_pid_freeze_active[FOC_DETAIL_LOG_SIZE];
extern volatile int8_t   g_foc_detail_log_zero_direction_pending[FOC_DETAIL_LOG_SIZE];
extern volatile int16_t  g_foc_detail_log_decel_hold_mA[FOC_DETAIL_LOG_SIZE];
extern volatile uint32_t g_foc_detail_log_edge_elapsed_us[FOC_DETAIL_LOG_SIZE];
extern volatile uint16_t g_foc_detail_log_fault[FOC_DETAIL_LOG_SIZE];
extern volatile uint8_t  g_foc_bidir_speed_enable;
extern volatile uint8_t  g_foc_bidir_speed_step_enable;
extern volatile uint8_t  g_foc_bidir_speed_reset_stats;
extern volatile uint32_t g_foc_bidir_speed_period_ms;
extern volatile uint16_t g_foc_bidir_speed_max_rpm;
extern volatile uint8_t  g_foc_bidir_speed_slew_enable;
extern volatile uint16_t g_foc_bidir_speed_slew_rpm_per_s;
extern volatile uint32_t g_foc_bidir_speed_elapsed_ms;
extern volatile uint16_t g_foc_bidir_speed_phase_u16;
extern volatile int16_t  g_foc_bidir_speed_raw_ref_rpm;
extern volatile int16_t  g_foc_bidir_speed_ref_rpm;
extern volatile uint8_t  g_foc_zero_transfer_enable;
extern volatile uint8_t  g_foc_zero_transfer_state;
extern volatile uint16_t g_foc_zero_transfer_enter_rpm;
extern volatile uint16_t g_foc_zero_transfer_exit_rpm;
extern volatile uint16_t g_foc_zero_transfer_ms;
extern volatile uint16_t g_foc_zero_relaunch_ms;
extern volatile uint32_t g_foc_zero_relaunch_edge_max_us;
extern volatile int16_t  g_foc_zero_hold_iq_mA;
extern volatile int16_t  g_foc_zero_breakaway_iq_mA;
extern volatile uint16_t g_foc_zero_iq_slew_mA_per_s;
extern volatile int16_t  g_foc_zero_signed_iq_cmd_mA;
extern volatile int16_t  g_foc_zero_iq_ff_mA;
extern volatile int16_t  g_foc_zero_ctrl_iq_raw_mA;
extern volatile uint8_t  g_foc_zero_pid_freeze_active;
extern volatile int8_t   g_foc_zero_direction_pending;
extern volatile uint16_t g_foc_zero_transfer_elapsed_ms;
extern volatile uint32_t g_foc_zero_edge_elapsed_us;
extern volatile uint32_t g_foc_zero_transfer_count;
extern volatile uint8_t  g_foc_zero_transfer_start_reason;
extern volatile int8_t   g_foc_zero_transfer_raw_sign;
extern volatile int8_t   g_foc_zero_transfer_prev_sign;
extern volatile uint8_t  g_foc_zero_transfer_raw_decreasing;
extern volatile uint8_t  g_foc_zero_transfer_cmd_decreasing;
extern volatile uint8_t  g_foc_zero_transfer_decel_to_zero;
extern volatile uint16_t g_foc_zero_transfer_raw_abs_rpm;
extern volatile uint16_t g_foc_zero_transfer_prev_cmd_abs_rpm;
extern volatile uint8_t  g_foc_bidir_zero_cross_enable;
extern volatile uint16_t g_foc_bidir_zero_speed_rpm;
extern volatile uint16_t g_foc_bidir_zero_confirm_ms;
extern volatile uint16_t g_foc_bidir_zero_hold_ms;
extern volatile uint16_t g_foc_bidir_zero_timeout_ms;
extern volatile uint16_t g_foc_bidir_zero_approach_start_rpm;
extern volatile uint16_t g_foc_bidir_zero_approach_slew_rpm_per_s;
extern volatile int16_t  g_foc_bidir_zero_approach_brake_limit_mA;
extern volatile uint16_t g_foc_bidir_zero_tail_start_rpm;
extern volatile uint16_t g_foc_bidir_zero_tail_slew_rpm_per_s;
extern volatile int16_t  g_foc_bidir_zero_tail_brake_limit_mA;
extern volatile uint8_t  g_foc_bidir_zero_tail_active;
extern volatile uint16_t g_foc_bidir_zero_tail_fdb_drop_rpm;
extern volatile uint16_t g_foc_bidir_zero_tail_fdb_lead_rpm;
extern volatile uint32_t g_foc_bidir_zero_tail_fdb_catch_count;
extern volatile uint16_t g_foc_bidir_zero_tail_min_drive_mA;
extern volatile uint16_t g_foc_bidir_zero_tail_drive_deadband_rpm;
extern volatile uint32_t g_foc_bidir_zero_tail_drive_assist_count;
extern volatile uint8_t  g_foc_bidir_zero_cross_state;
extern volatile uint32_t g_foc_bidir_zero_cross_count;
extern volatile uint16_t g_foc_bidir_zero_cross_elapsed_ms;
extern volatile uint16_t g_foc_bidir_zero_below_elapsed_ms;
extern volatile uint8_t  g_foc_bidir_zero_approach_active;
extern volatile int16_t  g_foc_bidir_zero_ref_rpm;
extern volatile uint32_t g_foc_bidir_zero_brake_limited_count;
extern volatile uint8_t  g_foc_observer_no_edge_active;

static uint16_t s_foc_log_decim = 0U;
static uint32_t s_start_log_start_us = 0U;
static uint32_t s_start_log_last_us = 0U;
static uint16_t s_hall_illegal_transition_count_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint32_t s_foc_prof_last_enter_us_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint32_t s_foc_control_period_us_store[FOC_CORE_MOTOR_COUNT] = {
    FOC_CONTROL_PERIOD_US,
    FOC_CONTROL_PERIOD_US
};
static uint16_t s_hall_recovery_accept_cycles_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint16_t s_post_recovery_duty_slew_cycles_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint16_t s_recovery_zero_vector_cycles_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint16_t s_recovery_zero_vector_min_cycles_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint32_t s_speed_loop_accum_us_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint16_t s_speed_drop_prev_abs_ref_rpm_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint16_t s_speed_drop_bidir_no_edge_holdoff_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint16_t s_speed_fdb_drop_peak_rpm_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static float s_speed_ref_ctrl_store[FOC_CORE_MOTOR_COUNT] = {0.0f, 0.0f};
static FOC_Dir_e s_speed_ref_ctrl_direction_store[FOC_CORE_MOTOR_COUNT] = {
    FOC_DIR_CW,
    FOC_DIR_CW
};
static float s_speed_error_boost_prev_ref_store[FOC_CORE_MOTOR_COUNT] = {0.0f, 0.0f};
static float s_bidir_decel_hold_prev_ref_store[FOC_CORE_MOTOR_COUNT] = {0.0f, 0.0f};
static float s_bidir_zero_soft_prev_ref_store[FOC_CORE_MOTOR_COUNT] = {0.0f, 0.0f};
static float s_current_angle_trim_rad_store[FOC_CORE_MOTOR_COUNT] = {0.0f, 0.0f};
static uint32_t s_hall_event_seq_seen_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};

#define s_hall_illegal_transition_count  (s_hall_illegal_transition_count_store[s_foc_core_active_motor])
#define s_foc_prof_last_enter_us         (s_foc_prof_last_enter_us_store[s_foc_core_active_motor])
#define s_foc_control_period_us          (s_foc_control_period_us_store[s_foc_core_active_motor])
#define s_hall_recovery_accept_cycles    (s_hall_recovery_accept_cycles_store[s_foc_core_active_motor])
#define s_post_recovery_duty_slew_cycles (s_post_recovery_duty_slew_cycles_store[s_foc_core_active_motor])
#define s_recovery_zero_vector_cycles    (s_recovery_zero_vector_cycles_store[s_foc_core_active_motor])
#define s_recovery_zero_vector_min_cycles (s_recovery_zero_vector_min_cycles_store[s_foc_core_active_motor])
#define s_speed_loop_accum_us            (s_speed_loop_accum_us_store[s_foc_core_active_motor])
#define s_speed_drop_prev_abs_ref_rpm    (s_speed_drop_prev_abs_ref_rpm_store[s_foc_core_active_motor])
#define s_speed_drop_bidir_no_edge_holdoff (s_speed_drop_bidir_no_edge_holdoff_store[s_foc_core_active_motor])
#define s_speed_fdb_drop_peak_rpm        (s_speed_fdb_drop_peak_rpm_store[s_foc_core_active_motor])
#define s_speed_ref_ctrl                 (s_speed_ref_ctrl_store[s_foc_core_active_motor])
#define s_speed_ref_ctrl_direction       (s_speed_ref_ctrl_direction_store[s_foc_core_active_motor])
#define s_speed_error_boost_prev_ref     (s_speed_error_boost_prev_ref_store[s_foc_core_active_motor])
#define s_bidir_decel_hold_prev_ref      (s_bidir_decel_hold_prev_ref_store[s_foc_core_active_motor])
#define s_bidir_zero_soft_prev_ref       (s_bidir_zero_soft_prev_ref_store[s_foc_core_active_motor])
#define s_current_angle_trim_rad         (s_current_angle_trim_rad_store[s_foc_core_active_motor])
#define s_hall_event_seq_seen            (s_hall_event_seq_seen_store[s_foc_core_active_motor])
#define s_hall_travel_stall_active       (s_foc_hall_travel_stall_active_store[s_foc_core_active_motor])
#define s_hall_travel_stall_counter      (s_foc_hall_travel_stall_counter_store[s_foc_core_active_motor])
#define s_hall_travel_stall_dir          (s_foc_hall_travel_stall_dir_store[s_foc_core_active_motor])
#define s_hall_travel_suppressed_delta   (s_foc_hall_travel_suppressed_delta_store[s_foc_core_active_motor])
static uint8_t s_dyn_speed_prev_enable = 0U;
static uint32_t s_dyn_speed_start_us = 0U;
static uint32_t s_dyn_log_last_us = 0U;
static float s_dyn_abs_err_avg_rpm = 0.0f;
static uint32_t s_detail_log_last_us = 0U;
static float s_detail_log_prev_abs_ref_rpm = 0.0f;
static int16_t s_detail_log_prev_raw_sign = 0;
static uint8_t s_detail_log_zero_event_seen = 0U;
static uint32_t s_detail_log_zero_event_us = 0U;
static uint8_t s_bidir_speed_prev_enable = 0U;
static uint32_t s_bidir_speed_start_us = 0U;
static uint32_t s_bidir_speed_last_us = 0U;
static float s_bidir_speed_limited_ref_rpm = 0.0f;
static uint8_t s_bidir_zero_state = 0U;
static uint32_t s_bidir_zero_start_us = 0U;
static uint32_t s_bidir_zero_hold_start_us = 0U;
static uint32_t s_bidir_zero_below_start_us = 0U;
static uint32_t s_bidir_zero_last_us = 0U;
static float s_bidir_zero_ref_rpm = 0.0f;
static int16_t s_bidir_zero_command_sign = 0;
static int16_t s_bidir_zero_pending_sign = 0;
static FOC_Dir_e s_bidir_zero_hold_dir = FOC_DIR_CW;
static uint8_t s_zero_transfer_state_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint32_t s_zero_transfer_start_us_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static uint32_t s_zero_transfer_last_us_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};
static float s_zero_transfer_prev_raw_store[FOC_CORE_MOTOR_COUNT] = {0.0f, 0.0f};
static float s_zero_transfer_signed_iq_store[FOC_CORE_MOTOR_COUNT] = {0.0f, 0.0f};
static float s_zero_transfer_target_iq_store[FOC_CORE_MOTOR_COUNT] = {0.0f, 0.0f};
static int16_t s_zero_transfer_old_sign_store[FOC_CORE_MOTOR_COUNT] = {0, 0};
static int16_t s_zero_transfer_new_sign_store[FOC_CORE_MOTOR_COUNT] = {0, 0};
static uint8_t s_zero_transfer_direction_switched_store[FOC_CORE_MOTOR_COUNT] = {0U, 0U};

#define s_zero_transfer_state       (s_zero_transfer_state_store[s_foc_core_active_motor])
#define s_zero_transfer_start_us    (s_zero_transfer_start_us_store[s_foc_core_active_motor])
#define s_zero_transfer_last_us     (s_zero_transfer_last_us_store[s_foc_core_active_motor])
#define s_zero_transfer_prev_raw    (s_zero_transfer_prev_raw_store[s_foc_core_active_motor])
#define s_zero_transfer_signed_iq   (s_zero_transfer_signed_iq_store[s_foc_core_active_motor])
#define s_zero_transfer_target_iq   (s_zero_transfer_target_iq_store[s_foc_core_active_motor])
#define s_zero_transfer_old_sign    (s_zero_transfer_old_sign_store[s_foc_core_active_motor])
#define s_zero_transfer_new_sign    (s_zero_transfer_new_sign_store[s_foc_core_active_motor])
#define s_zero_transfer_direction_switched \
    (s_zero_transfer_direction_switched_store[s_foc_core_active_motor])

#define FOC_BIDIR_ZERO_STATE_IDLE       0U
#define FOC_BIDIR_ZERO_STATE_DECEL      1U
#define FOC_BIDIR_ZERO_STATE_HOLD       2U

#define FOC_ZERO_TRANSFER_STATE_IDLE     0U
#define FOC_ZERO_TRANSFER_STATE_APPROACH 1U
#define FOC_ZERO_TRANSFER_STATE_TRANSFER 2U
#define FOC_ZERO_TRANSFER_STATE_RELAUNCH 3U

 

 /* ===================================================================

  *  内部函数声明

  * =================================================================== */

 

 static void FOC_Core_MainLoopOne(void);
 static void FOC_StateMachine(void);
 static uint32_t FOC_Prof_Enter(void);
 static void FOC_Prof_RecordSegment(uint32_t start_us, uint32_t end_us, uint8_t seg_id);
 static void FOC_Prof_Exit(uint32_t enter_us, uint32_t exit_us);
 static void FOC_Prof_Reset(void);
 static void FOC_LastFault_Reset(void);
 static uint32_t FOC_ControlPeriodUs(void);
 static uint8_t FOC_ControlPeriodIsLate(uint32_t period_us);
 static float FOC_ControlDtFromUs(uint32_t period_us, uint32_t max_us);
 static float FOC_ControlInvDtFromUs(uint32_t period_us, uint32_t max_us);
static uint8_t FOC_ControlPeriodNeedsRecovery(uint32_t period_us);
static void FOC_ResetClosedLoopForRecovery(void);
static void FOC_StartLog_Reset(void);
static void FOC_StartLog_Service(uint32_t now_us);
static void FOC_ResetCurrentAngleTrim(void);
static float FOC_ApplyCurrentAngleTrim(float theta_ctrl);
static void FOC_UpdateCurrentAngleTrim(float dt);
static void FOC_UpdateSpeedControlFeedback(void);
static void FOC_ResetSpeedDropFaultMonitor(void);
static void FOC_CheckSpeedDropFault(void);
static void FOC_ResetSpeedFdbDropFaultMonitor(void);
static void FOC_CheckSpeedFdbDropFault(void);
static uint8_t FOC_DynSpeed_HandleSetRef(float rpm);
static void FOC_DynSpeed_ServiceRef(void);
static int16_t FOC_BidirSpeed_TargetSign(float target);
static void FOC_BidirSpeed_ServiceRef(void);
static uint8_t FOC_BidirZeroTransfer_Active(void);
static void FOC_BidirZeroTransfer_Reset(void);
static float FOC_BidirZeroTransfer_ServiceRef(float target,
                                              float raw_target,
                                              uint32_t now_us,
                                              uint8_t raw_decel_to_zero,
                                              FOC_Dir_e *zero_dir);
static uint8_t FOC_BidirZeroTransfer_PidFrozen(void);
static float FOC_BidirZeroTransfer_ApplyIq(float iq_ref,
                                           float speed_dt);
static uint8_t FOC_IsAutoTestMotor(void);
static void FOC_DynSpeed_ServiceMetrics(void);
static void FOC_BeginRecoveryZeroVectorHold(void);
 static void FOC_ServiceRecoveryZeroVectorHold(void);
 static void FOC_BeginPostRecoveryDutySlew(void);
 static void FOC_RunRecoveryCurrentControl(float theta_ctrl,
                                           float pid_dt,
                                           float pid_inv_dt);
 static float FOC_LimitDutyStep(float target, float previous);
 static void FOC_ApplyPostRecoveryDutySlew(float prev_a,
                                           float prev_b,
                                           float prev_c);
 static void FOC_EnterFaultState(void);
static uint8_t FOC_HallSectorsAreAdjacent(uint8_t from, uint8_t to);
static int16_t FOC_ClampI32ToI16(int32_t v);
static void FOC_ResetHallTravelStallGuard(uint8_t clear_suppressed);
static void FOC_UpdateHallTravelStallGuard(void);
static void FOC_ApplyHallTravelStallCurrentCut(uint8_t reset_pid);
static uint8_t FOC_HallTravelStallTorqueBlocked(void);
static int32_t FOC_FilterHallTravelDelta(int32_t delta);
static int32_t FOC_RecordHallTravelStep(uint8_t prev_sector, uint8_t cur_sector);
static uint32_t FOC_HallMinSectorTimeUs(void);
static uint8_t FOC_ApplyHallSector(const FOC_HallSector_t *candidate,
                                    uint32_t timestamp_us,
                                    uint8_t timestamp_valid,
                                    uint8_t allow_missed_transition);

 static int16_t FOC_Log_ToI16(float v, float scale)
 {
     float x = v * scale;

     if (x > 32767.0f) {
         return 32767;
     }
     if (x < -32768.0f) {
         return -32768;
     }
     return (int16_t)x;
 }

 static uint16_t FOC_Log_ToU16(float v, float scale)
 {
     float x = v * scale;

     if (x > 65535.0f) {
         return 65535U;
     }
     if (x < 0.0f) {
         return 0U;
     }
     return (uint16_t)x;
 }

 static uint16_t FOC_Log_U32ToU16(uint32_t v)
 {
     return (v > 65535U) ? 65535U : (uint16_t)v;
 }

 static uint16_t FOC_Log_AngleU16(float angle)
 {
     while (angle >= FOC_2PI) {
         angle -= FOC_2PI;
     }
     while (angle < 0.0f) {
         angle += FOC_2PI;
     }
     return (uint16_t)(angle * (65535.0f / FOC_2PI));
 }

 void FOC_Core_SelectMotor(uint8_t motor_id)
 {
     if (motor_id >= FOC_CORE_MOTOR_COUNT) {
         motor_id = 0U;
     }

     s_foc_core_active_motor = motor_id;
     FOC_HAL_SelectMotor(motor_id);
 }

 uint8_t FOC_Core_GetSelectedMotor(void)
 {
     uint8_t motor = s_foc_core_active_motor;

     return (motor < FOC_CORE_MOTOR_COUNT) ? motor : 0U;
 }

 const FOC_Context_t *FOC_Core_GetContextByMotor(uint8_t motor_id)
 {
     if (motor_id >= FOC_CORE_MOTOR_COUNT) {
         motor_id = 0U;
     }

     return &g_foc_ctx[motor_id];
 }

static float FOC_ControlIqRef(void)
{
     return (s_ctx.direction == FOC_DIR_CCW) ? -s_ctx.iq_ref : s_ctx.iq_ref;
}

static float FOC_ApplyAppDirectionInvertToRef(float ref)
{
     return (g_foc_app_direction_invert_enable != 0U) ? -ref : ref;
}

static FOC_Dir_e FOC_ApplyAppDirectionInvertToDir(FOC_Dir_e dir)
{
     if (g_foc_app_direction_invert_enable == 0U) {
         return dir;
     }

     return (dir == FOC_DIR_CCW) ? FOC_DIR_CW : FOC_DIR_CCW;
}

static float FOC_SignedSpeedRef(void)
{
     float ref = FOC_FABS(s_ctx.speed_ref);

     return (s_ctx.direction == FOC_DIR_CCW) ? -ref : ref;
 }

static void FOC_StartLog_Reset(void)
{
     uint16_t i;

     g_foc_start_log_idx = 0U;
     g_foc_start_log_active = 0U;
     g_foc_start_log_stop = 0U;
     g_foc_start_log_armed = 1U;
     g_foc_start_log_trigger_motor_id = 0xFFU;
     g_foc_start_log_reset = 0U;
     s_start_log_start_us = 0U;
     s_start_log_last_us = 0U;

     for (i = 0U; i < FOC_START_LOG_SIZE; i++) {
         g_foc_start_log_valid[i] = 0U;
     }
}

static uint8_t FOC_StartLog_FilterMatches(void)
{
     uint8_t filter = g_foc_start_log_filter_motor_id;

     if (filter == 0xFFU) {
         return 1U;
     }

     return (filter == s_foc_core_active_motor) ? 1U : 0U;
}

static void FOC_StartLog_Record(uint32_t now_us)
{
     uint16_t idx = g_foc_start_log_idx;
     uint32_t decim_us = (uint32_t)g_foc_start_log_decim_ms * 1000U;
     float signed_ref = FOC_SignedSpeedRef();
     float signed_ctrl_ref = s_ctx.speed_ref_ctrl;
     float signed_fdb = s_ctx.speed_fdb;
     float signed_ctrl_fdb = s_ctx.speed_ctrl_fdb;
     float duty_max = s_ctx.duty_a;
     uint32_t edge_elapsed_us = 0U;
     uint8_t flags = 0U;

     if (idx >= FOC_START_LOG_SIZE) {
         g_foc_start_log_active = 0U;
         g_foc_start_log_stop = 1U;
         return;
     }

     if (decim_us == 0U) {
         decim_us = FOC_CONTROL_PERIOD_US;
     }
     if ((s_start_log_last_us != 0U) &&
         ((now_us - s_start_log_last_us) < decim_us)) {
         return;
     }

     if (s_ctx.direction == FOC_DIR_CCW) {
         signed_ctrl_ref = -signed_ctrl_ref;
         signed_fdb = -signed_fdb;
         signed_ctrl_fdb = -signed_ctrl_fdb;
     }
     if (s_ctx.timestamp_prev != 0U) {
         edge_elapsed_us = now_us - s_ctx.timestamp_prev;
     }
     if (s_ctx.duty_b > duty_max) {
         duty_max = s_ctx.duty_b;
     }
     if (s_ctx.duty_c > duty_max) {
         duty_max = s_ctx.duty_c;
     }
     if (s_ctx.direction == FOC_DIR_CCW) {
         flags |= 0x01U;
     }
     if (g_foc_speed_ref_ramp_active != 0U) {
         flags |= 0x02U;
     }
     if (g_foc_lift_current_limit_active != 0U) {
         flags |= 0x04U;
     }
     if (g_foc_low_speed_torque_active != 0U) {
         flags |= 0x08U;
     }
     if (g_foc_low_speed_iq_slew_active != 0U) {
         flags |= 0x10U;
     }
     if (s_foc_ctrl_source != FOC_CTRL_SOURCE_SPEED) {
         flags |= 0x20U;
     }
     if (s_ctx.fault != FOC_FAULT_NONE) {
         flags |= 0x40U;
     }

     g_foc_start_log_valid[idx] = 0U;
     g_foc_start_log_motor_id[idx] = s_foc_core_active_motor;
     g_foc_start_log_t_ms[idx] = (now_us - s_start_log_start_us) / 1000U;
     g_foc_start_log_ref_rpm[idx] = FOC_Log_ToI16(signed_ref, 1.0f);
     g_foc_start_log_ctrl_ref_rpm[idx] =
         FOC_Log_ToI16(signed_ctrl_ref, 1.0f);
     g_foc_start_log_fdb_rpm[idx] = FOC_Log_ToI16(signed_fdb, 1.0f);
     g_foc_start_log_ctrl_fdb_rpm[idx] =
         FOC_Log_ToI16(signed_ctrl_fdb, 1.0f);
     g_foc_start_log_err_rpm[idx] =
         FOC_Log_ToI16(signed_ctrl_ref - signed_ctrl_fdb, 1.0f);
     g_foc_start_log_iq_ref_mA[idx] =
         FOC_Log_ToI16(FOC_ControlIqRef(), 1000.0f);
     g_foc_start_log_iq_mA[idx] = FOC_Log_ToI16(s_ctx.i_dq.q, 1000.0f);
     g_foc_start_log_current_peak_mA[idx] =
         FOC_Log_ToU16(s_ctx.current_peak, 1000.0f);
     g_foc_start_log_vq_mV[idx] = FOC_Log_ToI16(s_ctx.v_dq.q, 1000.0f);
     g_foc_start_log_vbus_mV[idx] = FOC_Log_ToU16(s_ctx.v_bus, 1000.0f);
     g_foc_start_log_duty_max[idx] = FOC_Log_ToU16(duty_max, 10000.0f);
     g_foc_start_log_hall_sector[idx] = s_ctx.hall_sector.sector;
     g_foc_start_log_sector_no_change_count[idx] =
         FOC_Log_U32ToU16((uint32_t)s_ctx.sector_no_change_count);
     g_foc_start_log_edge_elapsed_us[idx] = edge_elapsed_us;
     g_foc_start_log_fault[idx] = (uint8_t)s_ctx.fault;
     g_foc_start_log_flags[idx] = flags;
     g_foc_start_log_lift_extra_mA[idx] =
         g_foc_lift_current_limit_extra_mA;
     g_foc_start_log_iq_slew_limited_mA[idx] =
         g_foc_low_speed_iq_slew_limited_mA;
     g_foc_start_log_valid[idx] = 1U;

     s_start_log_last_us = now_us;
     idx++;
     g_foc_start_log_idx = idx;
     if (idx >= FOC_START_LOG_SIZE) {
         g_foc_start_log_active = 0U;
         g_foc_start_log_stop = 1U;
     }
}

static void FOC_StartLog_Service(uint32_t now_us)
{
     float abs_ref;
     float abs_fdb;
     float trigger_ref = (float)g_foc_start_log_trigger_ref_rpm;
     float trigger_fdb = (float)g_foc_start_log_trigger_fdb_max_rpm;

     if (g_foc_start_log_reset != 0U) {
         FOC_StartLog_Reset();
     }
     if (g_foc_start_log_enable == 0U) {
         g_foc_start_log_active = 0U;
         return;
     }
     if (trigger_ref < 1.0f) {
         trigger_ref = 1.0f;
     }

     if (g_foc_start_log_active == 0U) {
         if ((g_foc_start_log_armed == 0U) ||
             (g_foc_start_log_stop != 0U) ||
             (FOC_StartLog_FilterMatches() == 0U) ||
             (s_foc_ctrl_source != FOC_CTRL_SOURCE_SPEED) ||
             (s_ctx.state != FOC_STATE_RUNNING)) {
             return;
         }

         abs_ref = FOC_FABS(s_ctx.speed_ref);
         abs_fdb = FOC_FABS(s_ctx.speed_fdb);
         if ((abs_ref < trigger_ref) || (abs_fdb > trigger_fdb)) {
             return;
         }

         g_foc_start_log_active = 1U;
         g_foc_start_log_armed = 0U;
         g_foc_start_log_stop = 0U;
         g_foc_start_log_idx = 0U;
         g_foc_start_log_trigger_motor_id = s_foc_core_active_motor;
         s_start_log_start_us = now_us;
         s_start_log_last_us = 0U;
         if (g_foc_start_log_trigger_count < 0xFFFFFFFFU) {
             g_foc_start_log_trigger_count++;
         }
     }

     if (s_foc_core_active_motor != g_foc_start_log_trigger_motor_id) {
         return;
     }

     FOC_StartLog_Record(now_us);
}

 static void FOC_ResetSpeedRefRamp(void)
 {
     s_speed_ref_ctrl = 0.0f;
     s_ctx.speed_ref_ctrl = 0.0f;
     s_speed_ref_ctrl_direction = s_ctx.direction;
     g_foc_speed_ref_cmd_rpm = FOC_Log_ToI16(FOC_FABS(s_ctx.speed_ref), 1.0f);
     g_foc_speed_ref_ctrl_rpm = 0;
     g_foc_speed_ref_ramp_active = 0U;
 }
 static void FOC_ResetSpeedLoopForZeroHold(void)
 {
     FOC_ResetSpeedRefRamp();
     FOC_PID_Reset(&s_ctx.pid_speed);
     FOC_PID_Reset(&s_ctx.pid_iq);
     s_ctx.iq_ref = 0.0f;
     s_ctx.speed_loop_counter = 0U;
     s_speed_loop_accum_us = 0U;
     s_speed_error_boost_prev_ref = 0.0f;
     g_foc_speed_error_boost_mA = 0;
     g_foc_low_speed_torque_active = 0U;
     g_foc_low_speed_torque_applied_mA = 0;
     g_foc_current_q_ff_mV = 0;
     g_foc_low_speed_iq_slew_active = 0U;
     g_foc_low_speed_iq_slew_limited_mA = 0;
     g_foc_lift_current_limit_active = 0U;
     g_foc_lift_current_limit_extra_mA = 0;
     g_foc_bidir_decel_hold_active = 0U;
     g_foc_bidir_decel_hold_applied_mA = 0;
     g_foc_bidir_decel_hold_raw_err_rpm = 0;
     s_bidir_decel_hold_prev_ref = 0.0f;
     g_foc_bidir_zero_soft_active = 0U;
     g_foc_bidir_zero_soft_scale_percent = 100U;
     g_foc_bidir_zero_soft_limited_mA = 0;
     FOC_BidirZeroTransfer_Reset();
     s_bidir_zero_soft_prev_ref = 0.0f;
 }

static int8_t FOC_HallTravelSignI32(int32_t value)
{
    if (value > 0) {
        return 1;
    }
    if (value < 0) {
        return -1;
    }
    return 0;
}

static int8_t FOC_HallTravelCommandSign(void)
{
    if (FOC_FABS(s_ctx.speed_ref) < 1.0f) {
        return 0;
    }

    return (s_ctx.direction == FOC_DIR_CCW) ? 1 : -1;
}

static void FOC_UpdateHallTravelStallDebug(void)
{
    uint8_t motor = s_foc_core_active_motor;

    if (motor >= FOC_CORE_MOTOR_COUNT) {
        return;
    }

    g_foc_hall_travel_stall_active[motor] = s_hall_travel_stall_active;
    g_foc_hall_travel_stall_counter[motor] = s_hall_travel_stall_counter;
    g_foc_hall_travel_stall_dir[motor] = s_hall_travel_stall_dir;
    g_foc_hall_travel_suppressed_delta[motor] =
        s_hall_travel_suppressed_delta;
}

static void FOC_ResetHallTravelStallGuard(uint8_t clear_suppressed)
{
    uint8_t motor = s_foc_core_active_motor;

    s_hall_travel_stall_active = 0U;
    s_hall_travel_stall_counter = 0U;
    s_hall_travel_stall_dir = 0;
    if (clear_suppressed != 0U) {
        s_hall_travel_suppressed_delta = 0;
        if (motor < FOC_CORE_MOTOR_COUNT) {
            g_foc_hall_travel_freeze_count[motor] = 0U;
        }
    }
    FOC_UpdateHallTravelStallDebug();
}

static void FOC_ApplyHallTravelStallCurrentCut(uint8_t reset_pid)
{
    s_ctx.iq_ref = 0.0f;
    g_foc_speed_error_boost_mA = 0;
    g_foc_low_speed_torque_active = 0U;
    g_foc_low_speed_torque_applied_mA = 0;
    g_foc_low_speed_iq_slew_active = 0U;
    g_foc_low_speed_iq_slew_limited_mA = 0;
    g_foc_lift_current_limit_active = 0U;
    g_foc_lift_current_limit_extra_mA = 0;

    if (reset_pid != 0U) {
        FOC_PID_Reset(&s_ctx.pid_speed);
        FOC_PID_Reset(&s_ctx.pid_iq);
        s_speed_loop_accum_us = 0U;
        s_ctx.speed_loop_counter = 0U;
    }
}

static void FOC_UpdateHallTravelStallGuard(void)
{
    float ref_abs = FOC_FABS(s_ctx.speed_ref);
    float ctrl_ref_abs = FOC_FABS(s_speed_ref_ctrl);
    float fdb_abs = FOC_FABS(s_ctx.speed_fdb);
    float current_abs = FOC_FABS(s_ctx.iq_ref);
    float iq_fdb_abs = FOC_FABS(s_ctx.i_dq.q);
    float err_abs;
    uint16_t threshold = g_foc_hall_travel_stall_count_threshold;
    uint16_t max_dir_counts = g_foc_hall_travel_stall_max_dir_counts;
    uint64_t direction_counts =
        s_foc_hall_direction_update_count[s_foc_core_active_motor];
    int8_t command_sign = FOC_HallTravelCommandSign();
    uint8_t was_active = s_hall_travel_stall_active;
    uint8_t low_progress = 0U;
    uint8_t condition = 0U;
    uint8_t keep_latched = 0U;

    if (ctrl_ref_abs > ref_abs) {
        ref_abs = ctrl_ref_abs;
    }
    if (iq_fdb_abs > current_abs) {
        current_abs = iq_fdb_abs;
    }
    if (s_ctx.current_peak > current_abs) {
        current_abs = s_ctx.current_peak;
    }

    err_abs = (ref_abs > fdb_abs) ? (ref_abs - fdb_abs) : (fdb_abs - ref_abs);
    if (fdb_abs <= (float)g_foc_hall_travel_stall_max_fdb_rpm) {
        low_progress = 1U;
    }
    if ((max_dir_counts != 0U) &&
        (direction_counts <= (uint64_t)max_dir_counts)) {
        low_progress = 1U;
    }

    if ((g_foc_hall_travel_stall_guard_enable != 0U) &&
        (g_foc_dyn_speed_enable == 0U) &&
        (g_foc_bidir_speed_enable == 0U) &&
        (s_ctx.state == FOC_STATE_RUNNING) &&
        (s_foc_ctrl_source == FOC_CTRL_SOURCE_SPEED) &&
        (ref_abs >= (float)g_foc_hall_travel_stall_min_ref_rpm) &&
        (low_progress != 0U) &&
        (err_abs >= (float)g_foc_hall_travel_stall_min_err_rpm) &&
        (current_abs >= ((float)g_foc_hall_travel_stall_min_iq_mA * 0.001f)) &&
        (command_sign != 0)) {
        condition = 1U;
    }

    if ((s_hall_travel_stall_active != 0U) &&
        (g_foc_hall_travel_stall_guard_enable != 0U) &&
        (g_foc_dyn_speed_enable == 0U) &&
        (g_foc_bidir_speed_enable == 0U) &&
        (ref_abs >= (float)g_foc_hall_travel_stall_min_ref_rpm) &&
        (command_sign != 0) &&
        ((s_hall_travel_stall_dir == 0) ||
         (command_sign == s_hall_travel_stall_dir))) {
        keep_latched = 1U;
    }

    if ((s_hall_travel_stall_dir != 0) &&
        (command_sign != 0) &&
        (command_sign != s_hall_travel_stall_dir)) {
        condition = 0U;
        keep_latched = 0U;
    }

    if ((s_hall_travel_suppressed_delta != 0) &&
        (command_sign != 0) &&
        (FOC_HallTravelSignI32(s_hall_travel_suppressed_delta) != command_sign)) {
        condition = 0U;
        keep_latched = 0U;
    }

    if (keep_latched != 0U) {
        s_hall_travel_stall_active = 1U;
        if (s_hall_travel_stall_dir == 0) {
            s_hall_travel_stall_dir = command_sign;
        }
        FOC_ApplyHallTravelStallCurrentCut((was_active == 0U) ? 1U : 0U);
        FOC_UpdateHallTravelStallDebug();
        return;
    }

    if (condition != 0U) {
        if (threshold == 0U) {
            threshold = 1U;
        }
        if (s_hall_travel_stall_counter < threshold) {
            s_hall_travel_stall_counter++;
        }
        if (s_hall_travel_stall_counter >= threshold) {
            s_hall_travel_stall_active = 1U;
            s_hall_travel_stall_dir = command_sign;
        } else {
            s_hall_travel_stall_active = 0U;
        }
    } else {
        s_hall_travel_stall_active = 0U;
        s_hall_travel_stall_counter = 0U;
        s_hall_travel_stall_dir = 0;
    }

    if (s_hall_travel_stall_active != 0U) {
        FOC_ApplyHallTravelStallCurrentCut((was_active == 0U) ? 1U : 0U);
    }

    FOC_UpdateHallTravelStallDebug();
}

static uint8_t FOC_HallTravelStallTorqueBlocked(void)
{
    int8_t command_sign;

    if (s_hall_travel_stall_active == 0U) {
        return 0U;
    }

    command_sign = FOC_HallTravelCommandSign();
    if ((command_sign != 0) &&
        (s_hall_travel_stall_dir != 0) &&
        (command_sign != s_hall_travel_stall_dir)) {
        return 0U;
    }

    return 1U;
}

static int32_t FOC_FilterHallTravelDelta(int32_t delta)
{
    int8_t delta_sign;
    int8_t suppressed_sign;

    if (delta == 0) {
        return 0;
    }

    delta_sign = FOC_HallTravelSignI32(delta);
    suppressed_sign = FOC_HallTravelSignI32(s_hall_travel_suppressed_delta);

    if (suppressed_sign != 0) {
        if (delta_sign != suppressed_sign) {
            int32_t remaining = s_hall_travel_suppressed_delta + delta;
            int8_t remaining_sign = FOC_HallTravelSignI32(remaining);

            if ((remaining_sign == 0) || (remaining_sign == suppressed_sign)) {
                s_hall_travel_suppressed_delta = remaining;
                FOC_UpdateHallTravelStallDebug();
                return 0;
            }

            s_hall_travel_suppressed_delta = 0;
            FOC_UpdateHallTravelStallDebug();
            return (s_hall_travel_stall_active != 0U) ? 0 : remaining;
        }

        if (s_hall_travel_stall_active == 0U) {
            s_hall_travel_suppressed_delta = 0;
            FOC_UpdateHallTravelStallDebug();
            return delta;
        }
    }

    if (s_hall_travel_stall_active != 0U) {
        uint8_t motor = s_foc_core_active_motor;

        s_hall_travel_suppressed_delta += delta;
        if (s_hall_travel_stall_dir == 0) {
            s_hall_travel_stall_dir = delta_sign;
        }
        if ((motor < FOC_CORE_MOTOR_COUNT) &&
            (g_foc_hall_travel_freeze_count[motor] < 0xFFFFFFFFU)) {
            g_foc_hall_travel_freeze_count[motor]++;
        }
        FOC_UpdateHallTravelStallDebug();
        return 0;
    }

    return delta;
}

static float FOC_ApplyBidirTailDriveAssist(float iq_ref,
                                           float speed_ref_ctrl,
                                           float speed_error)
{
     float min_drive =
         (float)g_foc_bidir_zero_tail_min_drive_mA * 0.001f;
     float deadband =
         (float)g_foc_bidir_zero_tail_drive_deadband_rpm;
     float zero_speed = (float)g_foc_bidir_zero_speed_rpm;
     float tail_start = (float)g_foc_bidir_zero_tail_start_rpm;
     float tail_range;
     float drive_scale;

     if ((g_foc_bidir_speed_enable == 0U) ||
         (g_foc_bidir_zero_tail_active == 0U) ||
         (g_foc_bidir_zero_cross_state == FOC_BIDIR_ZERO_STATE_HOLD) ||
         (min_drive <= 0.0f) ||
         (speed_ref_ctrl <= zero_speed) ||
         (speed_error < -deadband)) {
         return iq_ref;
     }

     if (tail_start <= zero_speed) {
         tail_start = zero_speed + 1.0f;
     }
     tail_range = tail_start - zero_speed;
     drive_scale = (speed_ref_ctrl - zero_speed) / tail_range;
     drive_scale = FOC_CLAMP(drive_scale, 0.0f, 1.0f);
     min_drive *= drive_scale;

     if (iq_ref < min_drive) {
         if (g_foc_bidir_zero_tail_drive_assist_count < 0xFFFFFFFFU) {
             g_foc_bidir_zero_tail_drive_assist_count++;
         }
         return min_drive;
     }

     return iq_ref;
}

static float FOC_ApplyBidirDecelHoldAssist(float iq_ref,
                                           float speed_ref_ctrl)
{
     float prev_ref = s_bidir_decel_hold_prev_ref;
     float raw_speed_error = speed_ref_ctrl - s_ctx.speed_fdb;
     float min_iq =
         (float)g_foc_bidir_decel_hold_min_iq_mA * 0.001f;
     float max_rpm = (float)g_foc_bidir_decel_hold_max_rpm;
     float full_rpm = (float)g_foc_bidir_decel_hold_full_rpm;
     float err_rpm = (float)g_foc_bidir_decel_hold_err_rpm;
     float speed_scale;
     float err_scale;
     uint8_t ref_decreasing =
         ((prev_ref - speed_ref_ctrl) > 0.5f) ? 1U : 0U;

     g_foc_bidir_decel_hold_active = 0U;
     g_foc_bidir_decel_hold_applied_mA = 0;
     g_foc_bidir_decel_hold_raw_err_rpm =
         FOC_Log_ToI16(raw_speed_error, 1.0f);
     s_bidir_decel_hold_prev_ref = speed_ref_ctrl;

     if ((g_foc_bidir_decel_hold_enable == 0U) ||
         (g_foc_bidir_speed_enable == 0U) ||
         (ref_decreasing == 0U) ||
         (min_iq <= 0.0f) ||
         (max_rpm < 1.0f) ||
         (speed_ref_ctrl < 1.0f) ||
         (speed_ref_ctrl > max_rpm) ||
         (raw_speed_error <= err_rpm) ||
         (iq_ref < 0.0f)) {
         return iq_ref;
     }

     if (full_rpm < 1.0f) {
         full_rpm = max_rpm;
     }
     if (err_rpm < 1.0f) {
         err_rpm = 1.0f;
     }

     speed_scale = FOC_CLAMP(speed_ref_ctrl / full_rpm, 0.0f, 1.0f);
     err_scale =
         FOC_CLAMP((raw_speed_error - err_rpm) / err_rpm, 0.0f, 1.0f);
     min_iq *= speed_scale * err_scale;

     if (iq_ref < min_iq) {
         g_foc_bidir_decel_hold_active = 1U;
         g_foc_bidir_decel_hold_applied_mA =
             FOC_Log_ToI16(min_iq - iq_ref, 1000.0f);
         if (g_foc_bidir_decel_hold_count < 0xFFFFFFFFU) {
             g_foc_bidir_decel_hold_count++;
         }
         return min_iq;
     }

     return iq_ref;
}

static float FOC_ApplyBidirZeroSoftLanding(float iq_ref,
                                           float speed_ref_ctrl)
{
    float prev_ref = s_bidir_zero_soft_prev_ref;
    float start_rpm = (float)g_foc_bidir_zero_soft_start_rpm;
    float ref_scale;
    float limited_iq;
    uint8_t ref_decreasing =
        ((prev_ref - speed_ref_ctrl) > 0.5f) ? 1U : 0U;

    g_foc_bidir_zero_soft_active = 0U;
    g_foc_bidir_zero_soft_scale_percent = 100U;
    g_foc_bidir_zero_soft_limited_mA = 0;
    s_bidir_zero_soft_prev_ref = speed_ref_ctrl;

    if ((g_foc_bidir_zero_soft_enable == 0U) ||
        (g_foc_bidir_speed_enable == 0U) ||
        (g_foc_bidir_speed_step_enable != 2U) ||
       (ref_decreasing == 0U) ||
        (start_rpm < 1.0f) ||
        (speed_ref_ctrl >= start_rpm)) {
        return iq_ref;
    }

    ref_scale = FOC_CLAMP(speed_ref_ctrl / start_rpm, 0.0f, 1.0f);
    ref_scale *= ref_scale;
    g_foc_bidir_zero_soft_scale_percent =
        FOC_Log_ToU16(ref_scale, 100.0f);

    limited_iq = iq_ref * ref_scale;
    if (FOC_FABS(iq_ref - limited_iq) > 0.001f) {
        g_foc_bidir_zero_soft_active = 1U;
        g_foc_bidir_zero_soft_limited_mA =
            FOC_Log_ToI16(FOC_FABS(iq_ref - limited_iq), 1000.0f);
        if (g_foc_bidir_zero_soft_count < 0xFFFFFFFFU) {
            g_foc_bidir_zero_soft_count++;
        }
    }

    return limited_iq;
}

 static void FOC_NormalizeSignedSpeedRef(void)
 {
     uint8_t reset_loop = 0U;

     if (s_ctx.speed_ref < 0.0f) {
         s_ctx.speed_ref = -s_ctx.speed_ref;
         if (s_ctx.direction != FOC_DIR_CCW) {
             s_ctx.direction = FOC_DIR_CCW;
             reset_loop = 1U;
         }
         g_foc_signed_speed_ref_normalize_count++;
     } else if (s_ctx.direction > FOC_DIR_CCW) {
         s_ctx.direction = FOC_DIR_CW;
         reset_loop = 1U;
     }

     if (s_ctx.speed_ref > s_config.motor.max_speed_rpm) {
         s_ctx.speed_ref = s_config.motor.max_speed_rpm;
     }

     if (reset_loop != 0U) {
         FOC_PID_Reset(&s_ctx.pid_speed);
         FOC_PID_Reset(&s_ctx.pid_iq);
         s_ctx.iq_ref = 0.0f;
         s_ctx.speed_loop_counter = 0U;
         s_speed_loop_accum_us = 0U;
         s_speed_error_boost_prev_ref = 0.0f;
         FOC_ResetSpeedRefRamp();
     }
 }

static float FOC_UpdateSpeedRefRamp(uint32_t dt_us)
{
     float target = FOC_FABS(s_ctx.speed_ref);
     (void)dt_us;

     if (target > s_config.motor.max_speed_rpm) {
         target = s_config.motor.max_speed_rpm;
     }

     if (s_speed_ref_ctrl_direction != s_ctx.direction) {
         s_speed_ref_ctrl_direction = s_ctx.direction;
         FOC_PID_Reset(&s_ctx.pid_speed);
         s_speed_error_boost_prev_ref = 0.0f;
     }

     s_speed_ref_ctrl = target;

     s_ctx.speed_ref_ctrl = s_speed_ref_ctrl;
     g_foc_speed_ref_cmd_rpm = FOC_Log_ToI16(target, 1.0f);
     g_foc_speed_ref_ctrl_rpm = FOC_Log_ToI16(s_speed_ref_ctrl, 1.0f);
     g_foc_speed_ref_ramp_active = 0U;

     return s_speed_ref_ctrl;
 }

 static float FOC_LimitRegenBrakingIq(float iq_ref)
 {
     float start_v = FOC_REGEN_BRAKE_LIMIT_START_V;
     float disable_v = FOC_REGEN_BRAKE_DISABLE_V;
     float full_brake_min = s_ctx.pid_speed.out_min;
     float min_allowed = full_brake_min;

     if (full_brake_min > 0.0f) {
         full_brake_min = 0.0f;
     }

     if (disable_v <= start_v) {
         disable_v = start_v + 0.1f;
     }

     if ((iq_ref >= 0.0f) || (s_ctx.v_bus < start_v)) {
         g_foc_vbus_brake_limit_mA = FOC_Log_ToI16(full_brake_min, 1000.0f);
         g_foc_vbus_brake_active = 0U;
         return iq_ref;
     }

     if (s_ctx.v_bus >= disable_v) {
         min_allowed = 0.0f;
     } else {
         float ratio = (disable_v - s_ctx.v_bus) / (disable_v - start_v);
         ratio = FOC_CLAMP(ratio, 0.0f, 1.0f);
         min_allowed = full_brake_min * ratio;
     }

     g_foc_vbus_brake_limit_mA = FOC_Log_ToI16(min_allowed, 1000.0f);

     if (iq_ref < min_allowed) {
         g_foc_vbus_brake_active = 1U;
         g_foc_vbus_brake_limited_count++;
         FOC_PID_Reset(&s_ctx.pid_speed);
         FOC_PID_Reset(&s_ctx.pid_iq);
         return min_allowed;
     }

     g_foc_vbus_brake_active = 0U;
     return iq_ref;
 }

 static float FOC_ApplyLowSpeedTorqueAssist(float iq_ref,
                                            float speed_ref_ctrl,
                                            float speed_error)
 {
     float min_iq = (float)g_foc_low_speed_torque_min_iq_mA * 0.001f;
     float max_rpm = (float)g_foc_low_speed_torque_max_rpm;
     float full_rpm = (float)g_foc_low_speed_torque_full_rpm;
     float err_rpm = (float)g_foc_low_speed_torque_err_rpm;
     float speed_scale;
     float err_scale;

     g_foc_low_speed_torque_active = 0U;
     g_foc_low_speed_torque_applied_mA = 0;

     if ((g_foc_low_speed_torque_enable == 0U) ||
         (min_iq <= 0.0f) ||
         (max_rpm < 1.0f) ||
         (speed_ref_ctrl < 1.0f) ||
         (speed_ref_ctrl > max_rpm) ||
         (speed_error <= err_rpm) ||
         (iq_ref < 0.0f)) {
         return iq_ref;
     }

     if (full_rpm < 1.0f) {
         full_rpm = max_rpm;
     }
     if (err_rpm < 1.0f) {
         err_rpm = 1.0f;
     }
     speed_scale = FOC_CLAMP(speed_ref_ctrl / full_rpm, 0.0f, 1.0f);
     err_scale = FOC_CLAMP((speed_error - err_rpm) / err_rpm, 0.0f, 1.0f);
     min_iq *= speed_scale * err_scale;

     if (iq_ref < min_iq) {
         g_foc_low_speed_torque_active = 1U;
         g_foc_low_speed_torque_applied_mA =
             FOC_Log_ToI16(min_iq - iq_ref, 1000.0f);
         return min_iq;
     }

     return iq_ref;
 }

 static float FOC_GetSpeedIqPositiveLimit(float speed_ref_ctrl,
                                          float speed_error)
 {
     float base_limit =
         (float)g_foc_lift_current_limit_base_mA * 0.001f;
     float boost_limit =
         (float)g_foc_lift_current_limit_boost_mA * 0.001f;
     float max_rpm = (float)g_foc_lift_current_limit_max_rpm;
     float err_rpm = (float)g_foc_lift_current_limit_err_rpm;
     FOC_Dir_e lift_dir =
         (g_foc_lift_current_limit_dir > FOC_DIR_CCW) ?
         FOC_DIR_CW : (FOC_Dir_e)g_foc_lift_current_limit_dir;

     g_foc_lift_current_limit_active = 0U;
     g_foc_lift_current_limit_extra_mA = 0;

     if (base_limit <= 0.0f) {
         base_limit = s_ctx.pid_speed.out_max;
     }
     if (boost_limit < base_limit) {
         boost_limit = base_limit;
     }
     if (max_rpm < 1.0f) {
         max_rpm = 1.0f;
     }
     if (err_rpm < 1.0f) {
         err_rpm = 1.0f;
     }

     if ((g_foc_lift_current_limit_enable != 0U) &&
         (s_ctx.direction == lift_dir) &&
         (speed_ref_ctrl >= 1.0f) &&
         (s_ctx.speed_fdb <= max_rpm) &&
         (speed_error >= err_rpm)) {
         g_foc_lift_current_limit_active = 1U;
         g_foc_lift_current_limit_extra_mA =
             FOC_Log_ToI16(boost_limit - base_limit, 1000.0f);
         return boost_limit;
     }

     return base_limit;
 }

static float FOC_ApplyLowSpeedIqSlew(float iq_ref,
                                     float speed_ref_ctrl,
                                     float speed_dt)
{
     (void)speed_ref_ctrl;
     (void)speed_dt;

     g_foc_low_speed_iq_slew_active = 0U;
     g_foc_low_speed_iq_slew_limited_mA = 0;

     return iq_ref;
}

 static float FOC_ApplyLowSpeedCurrentFeedForward(float vq)
 {
     float ref_abs = FOC_FABS(s_ctx.speed_ref_ctrl);
     float max_rpm = (float)g_foc_low_speed_current_ff_max_rpm;
     float gain = (float)g_foc_current_q_rs_ff_gain_milli * 0.001f;
     float ff_v = 0.0f;

     g_foc_current_q_ff_mV = 0;

     if ((g_foc_low_speed_current_ff_enable == 0U) ||
         (s_foc_ctrl_source != FOC_CTRL_SOURCE_SPEED) ||
         (max_rpm < 1.0f) ||
         (ref_abs > max_rpm) ||
         (gain <= 0.0f)) {
         return vq;
     }

     ff_v = s_config.motor.rs * FOC_ControlIqRef() * gain;
     g_foc_current_q_ff_mV = FOC_Log_ToI16(ff_v, 1000.0f);

     return FOC_CLAMP(vq + ff_v,
                      s_ctx.pid_iq.out_min,
                      s_ctx.pid_iq.out_max);
 }

static uint8_t FOC_DynSpeed_Near(float a, float b)
{
    return (FOC_FABS(a - b) < 0.5f) ? 1U : 0U;
}

static uint8_t FOC_DynSpeed_IsStartCommand(float rpm)
{
    if (FOC_DynSpeed_Near(rpm, (float)g_foc_dyn_speed_start_cmd_rpm) != 0U) {
        return 1U;
    }

    if ((g_foc_dyn_speed_start_on_max_ref != 0U) &&
        (FOC_DynSpeed_Near(rpm, (float)g_foc_dyn_speed_max_rpm) != 0U)) {
        return 1U;
    }

    return 0U;
}

static void FOC_DynSpeed_ResetLog(void)
{
    uint16_t i;

    g_foc_dyn_log_idx = 0U;
    g_foc_dyn_log_stop = 0U;
    s_dyn_log_last_us = 0U;

    for (i = 0U; i < FOC_DYN_SPEED_LOG_SIZE; i++) {
        g_foc_dyn_log_t_ms[i] = 0U;
        g_foc_dyn_log_ref_rpm[i] = 0;
        g_foc_dyn_log_fdb_rpm[i] = 0;
        g_foc_dyn_log_ctrl_fdb_rpm[i] = 0;
        g_foc_dyn_log_err_rpm[i] = 0;
        g_foc_dyn_log_iq_ref_mA[i] = 0;
        g_foc_dyn_log_iq_mA[i] = 0;
        g_foc_dyn_log_current_peak_mA[i] = 0U;
        g_foc_dyn_log_fault[i] = 0U;
    }
}

static void FOC_DetailLog_Reset(void)
{
    uint16_t i;

    g_foc_detail_log_idx = 0U;
    g_foc_detail_log_active = 0U;
    g_foc_detail_log_stop = (g_foc_detail_log_enable != 0U) ? 0U : 1U;
    g_foc_detail_log_armed = (g_foc_detail_log_enable != 0U) ? 1U : 0U;
    g_foc_detail_log_reset = 0U;
    s_detail_log_last_us = 0U;
    s_detail_log_prev_abs_ref_rpm = 0.0f;
    s_detail_log_prev_raw_sign = 0;
    s_detail_log_zero_event_seen = 0U;
    s_detail_log_zero_event_us = 0U;
    g_foc_detail_log_zero_event_idx = 0xFFFFU;
    g_foc_detail_log_zero_window_done = 0U;

    for (i = 0U; i < FOC_DETAIL_LOG_SIZE; i++) {
        g_foc_detail_log_t_ms[i] = 0U;
        g_foc_detail_log_raw_ref_rpm[i] = 0;
        g_foc_detail_log_ref_rpm[i] = 0;
        g_foc_detail_log_fdb_rpm[i] = 0;
        g_foc_detail_log_ctrl_fdb_rpm[i] = 0;
        g_foc_detail_log_iq_ref_mA[i] = 0;
        g_foc_detail_log_id_mA[i] = 0;
        g_foc_detail_log_iq_mA[i] = 0;
        g_foc_detail_log_current_peak_mA[i] = 0U;
        g_foc_detail_log_hall_raw[i] = 0U;
        g_foc_detail_log_hall_sector[i] = 0U;
        g_foc_detail_log_sector_no_change_count[i] = 0U;
        g_foc_detail_log_theta_hall[i] = 0U;
        g_foc_detail_log_theta_pred[i] = 0U;
        g_foc_detail_log_theta_ctrl[i] = 0U;
        g_foc_detail_log_duty_a[i] = 0U;
        g_foc_detail_log_duty_b[i] = 0U;
        g_foc_detail_log_duty_c[i] = 0U;
        g_foc_detail_log_zero_soft_scale_percent[i] = 0U;
        g_foc_detail_log_zero_soft_limited_mA[i] = 0;
        g_foc_detail_log_zero_soft_active[i] = 0U;
        g_foc_detail_log_zero_transfer_state[i] = 0U;
        g_foc_detail_log_zero_signed_iq_cmd_mA[i] = 0;
        g_foc_detail_log_zero_iq_ff_mA[i] = 0;
        g_foc_detail_log_zero_ctrl_iq_raw_mA[i] = 0;
        g_foc_detail_log_zero_pid_freeze_active[i] = 0U;
        g_foc_detail_log_zero_direction_pending[i] = 0;
        g_foc_detail_log_decel_hold_mA[i] = 0;
        g_foc_detail_log_edge_elapsed_us[i] = 0U;
        g_foc_detail_log_fault[i] = 0U;
    }
}

static void FOC_DetailLog_Record(uint32_t now_us, float theta_ctrl)
{
    uint16_t idx;
    uint32_t decim_us = (uint32_t)g_foc_detail_log_decim_ms * 1000U;
    uint32_t edge_elapsed_us = 0U;
    float signed_speed_fdb = s_ctx.speed_fdb;
    float signed_speed_ctrl_fdb = s_ctx.speed_ctrl_fdb;

    if (g_foc_detail_log_active == 0U) {
        return;
    }
    if (decim_us < 1000U) {
        decim_us = 1000U;
    }
    if ((s_detail_log_last_us != 0U) &&
        ((now_us - s_detail_log_last_us) < decim_us)) {
        return;
    }

    idx = g_foc_detail_log_idx;
    if (idx >= FOC_DETAIL_LOG_SIZE) {
        g_foc_detail_log_active = 0U;
        g_foc_detail_log_stop = 1U;
        return;
    }

    if ((g_foc_dyn_speed_ref_rpm < 0) &&
        (s_ctx.direction == FOC_DIR_CCW)) {
        signed_speed_fdb = -signed_speed_fdb;
        signed_speed_ctrl_fdb = -signed_speed_ctrl_fdb;
    }
    if (s_ctx.timestamp_prev != 0U) {
        edge_elapsed_us = now_us - s_ctx.timestamp_prev;
    }

    s_detail_log_last_us = now_us;
    g_foc_detail_log_t_ms[idx] = g_foc_bidir_speed_elapsed_ms;
    g_foc_detail_log_raw_ref_rpm[idx] = g_foc_bidir_speed_raw_ref_rpm;
    g_foc_detail_log_ref_rpm[idx] = g_foc_dyn_speed_ref_rpm;
    g_foc_detail_log_fdb_rpm[idx] =
        FOC_Log_ToI16(signed_speed_fdb, 1.0f);
    g_foc_detail_log_ctrl_fdb_rpm[idx] =
        FOC_Log_ToI16(signed_speed_ctrl_fdb, 1.0f);
    g_foc_detail_log_iq_ref_mA[idx] =
        FOC_Log_ToI16(FOC_ControlIqRef(), 1000.0f);
    g_foc_detail_log_id_mA[idx] = FOC_Log_ToI16(s_ctx.i_dq.d, 1000.0f);
    g_foc_detail_log_iq_mA[idx] = FOC_Log_ToI16(s_ctx.i_dq.q, 1000.0f);
    g_foc_detail_log_current_peak_mA[idx] =
        FOC_Log_ToU16(s_ctx.current_peak, 1000.0f);
    g_foc_detail_log_hall_raw[idx] =
        (uint8_t)((s_ctx.hall_raw.h1 << 2) |
                  (s_ctx.hall_raw.h2 << 1) |
                   s_ctx.hall_raw.h3);
    g_foc_detail_log_hall_sector[idx] = s_ctx.hall_sector.sector;
    g_foc_detail_log_sector_no_change_count[idx] =
        FOC_Log_U32ToU16((uint32_t)s_ctx.sector_no_change_count);
    g_foc_detail_log_theta_hall[idx] = FOC_Log_AngleU16(s_ctx.theta_e);
    g_foc_detail_log_theta_pred[idx] =
        FOC_Log_AngleU16(s_ctx.theta_e_predicted);
    g_foc_detail_log_theta_ctrl[idx] = FOC_Log_AngleU16(theta_ctrl);
    g_foc_detail_log_duty_a[idx] = FOC_Log_ToU16(s_ctx.duty_a, 10000.0f);
    g_foc_detail_log_duty_b[idx] = FOC_Log_ToU16(s_ctx.duty_b, 10000.0f);
    g_foc_detail_log_duty_c[idx] = FOC_Log_ToU16(s_ctx.duty_c, 10000.0f);
    g_foc_detail_log_zero_soft_scale_percent[idx] =
        g_foc_bidir_zero_soft_scale_percent;
    g_foc_detail_log_zero_soft_limited_mA[idx] =
        g_foc_bidir_zero_soft_limited_mA;
    g_foc_detail_log_zero_soft_active[idx] = g_foc_bidir_zero_soft_active;
    g_foc_detail_log_zero_transfer_state[idx] =
        g_foc_zero_transfer_state;
    g_foc_detail_log_zero_signed_iq_cmd_mA[idx] =
        g_foc_zero_signed_iq_cmd_mA;
    g_foc_detail_log_zero_iq_ff_mA[idx] =
        g_foc_zero_iq_ff_mA;
    g_foc_detail_log_zero_ctrl_iq_raw_mA[idx] =
        g_foc_zero_ctrl_iq_raw_mA;
    g_foc_detail_log_zero_pid_freeze_active[idx] =
        g_foc_zero_pid_freeze_active;
    g_foc_detail_log_zero_direction_pending[idx] =
        g_foc_zero_direction_pending;
    g_foc_detail_log_decel_hold_mA[idx] =
        g_foc_bidir_decel_hold_applied_mA;
    g_foc_detail_log_edge_elapsed_us[idx] = edge_elapsed_us;
    g_foc_detail_log_fault[idx] = (uint16_t)s_ctx.fault;

    idx++;
    g_foc_detail_log_idx = idx;
    if (idx >= FOC_DETAIL_LOG_SIZE) {
        g_foc_detail_log_active = 0U;
        g_foc_detail_log_stop = 1U;
    }
}

static void FOC_DetailLog_Service(float theta_ctrl, uint32_t now_us)
{
    float ref_abs = FOC_FABS((float)g_foc_dyn_speed_ref_rpm);
    uint8_t ref_decreasing =
        ((s_detail_log_prev_abs_ref_rpm - ref_abs) > 0.5f) ? 1U : 0U;
    int16_t raw_sign =
        FOC_BidirSpeed_TargetSign((float)g_foc_bidir_speed_raw_ref_rpm);
    uint8_t zero_event = 0U;
    uint8_t bidir_zero_window_mode =
        ((g_foc_bidir_speed_step_enable == 0U) ||
         (g_foc_bidir_speed_step_enable == 2U)) ? 1U : 0U;

    if (FOC_IsAutoTestMotor() == 0U) {
        return;
    }

    if (g_foc_detail_log_reset != 0U) {
        FOC_DetailLog_Reset();
    }
    if (g_foc_detail_log_enable == 0U) {
        g_foc_detail_log_active = 0U;
        g_foc_detail_log_armed = 0U;
        g_foc_detail_log_stop = 1U;
        s_detail_log_prev_abs_ref_rpm = ref_abs;
        return;
    }

    if ((g_foc_detail_log_armed != 0U) &&
        (g_foc_detail_log_active == 0U) &&
        (g_foc_detail_log_stop == 0U) &&
        (g_foc_bidir_speed_enable != 0U) &&
        (bidir_zero_window_mode != 0U) &&
        (ref_decreasing != 0U) &&
        (ref_abs <= (float)g_foc_detail_log_trigger_rpm) &&
        (ref_abs >= 1.0f)) {
        g_foc_detail_log_idx = 0U;
        g_foc_detail_log_active = 1U;
        g_foc_detail_log_armed = 0U;
        s_detail_log_zero_event_seen = 0U;
        s_detail_log_zero_event_us = 0U;
        g_foc_detail_log_zero_event_idx = 0xFFFFU;
        g_foc_detail_log_zero_window_done = 0U;
        s_detail_log_last_us = 0U;
        if (g_foc_detail_log_trigger_count < 0xFFFFFFFFU) {
            g_foc_detail_log_trigger_count++;
        }
    }

    if ((g_foc_detail_log_zero_window_enable != 0U) &&
        (g_foc_detail_log_active != 0U) &&
        (s_detail_log_zero_event_seen == 0U)) {
        if ((s_detail_log_prev_raw_sign != 0) &&
            (raw_sign != 0) &&
            (raw_sign != s_detail_log_prev_raw_sign)) {
            zero_event = 1U;
        }

        if (zero_event != 0U) {
            s_detail_log_zero_event_seen = 1U;
            s_detail_log_zero_event_us = now_us;
            g_foc_detail_log_zero_event_idx = g_foc_detail_log_idx;
            if (g_foc_detail_log_zero_event_count < 0xFFFFFFFFU) {
                g_foc_detail_log_zero_event_count++;
            }
        }
    }

    FOC_DetailLog_Record(now_us, theta_ctrl);
    if ((g_foc_detail_log_zero_window_enable != 0U) &&
        (g_foc_detail_log_active != 0U) &&
        (s_detail_log_zero_event_seen != 0U)) {
        uint32_t post_us = (uint32_t)g_foc_detail_log_zero_post_ms * 1000U;

        if (post_us == 0U) {
            post_us = 1U;
        }
        if ((now_us - s_detail_log_zero_event_us) >= post_us) {
            g_foc_detail_log_active = 0U;
            g_foc_detail_log_stop = 1U;
            g_foc_detail_log_zero_window_done = 1U;
        }
    }
    s_detail_log_prev_abs_ref_rpm = ref_abs;
    if (raw_sign != 0) {
        s_detail_log_prev_raw_sign = raw_sign;
    }
}

static void FOC_DynSpeed_ResetStats(uint32_t now_us)
{
    g_foc_dyn_speed_elapsed_ms = 0U;
    g_foc_dyn_speed_phase_u16 = 0U;
    g_foc_dyn_speed_ref_rpm = 0;
    g_foc_dyn_speed_fdb_rpm = 0;
    g_foc_dyn_speed_ctrl_fdb_rpm = 0;
    g_foc_dyn_speed_err_rpm = 0;
    g_foc_dyn_speed_abs_err_rpm = 0U;
    g_foc_dyn_speed_abs_err_avg_rpm = 0U;
    g_foc_dyn_speed_max_abs_err_rpm = 0U;
    g_foc_dyn_speed_iq_ref_mA = 0;
    g_foc_dyn_speed_id_mA = 0;
    g_foc_dyn_speed_iq_mA = 0;
    g_foc_dyn_speed_current_peak_mA = 0U;
    g_foc_dyn_speed_sample_count = 0U;
    g_foc_bidir_speed_raw_ref_rpm = 0;
    g_foc_bidir_speed_ref_rpm = 0;
    FOC_BidirZeroTransfer_Reset();
    g_foc_bidir_decel_hold_active = 0U;
    g_foc_bidir_decel_hold_applied_mA = 0;
    g_foc_bidir_decel_hold_raw_err_rpm = 0;
    g_foc_bidir_decel_hold_count = 0U;
    s_bidir_decel_hold_prev_ref = 0.0f;
    g_foc_bidir_zero_soft_active = 0U;
    g_foc_bidir_zero_soft_scale_percent = 100U;
    g_foc_bidir_zero_soft_limited_mA = 0;
    g_foc_bidir_zero_soft_count = 0U;
    s_bidir_zero_soft_prev_ref = 0.0f;
    s_dyn_abs_err_avg_rpm = 0.0f;
    s_dyn_speed_start_us = now_us;
    FOC_DynSpeed_ResetLog();
    FOC_DetailLog_Reset();
}

static void FOC_DynSpeed_ResetStatsAtMax(uint32_t now_us)
{
    uint32_t period_ms = g_foc_dyn_speed_period_ms;

    FOC_DynSpeed_ResetStats(now_us);
    if (period_ms < 100U) {
        period_ms = 100U;
    }
    s_dyn_speed_start_us = now_us - ((period_ms * 1000U) / 2U);
}

static float FOC_DynSpeed_CalcRef(uint32_t now_us)
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
    elapsed_us = now_us - s_dyn_speed_start_us;
    phase_us = (period_us > 0U) ? (elapsed_us % period_us) : 0U;
    phase = ((float)phase_us / (float)period_us) * FOC_2PI;

    speed_mid = 0.5f * (speed_min + speed_max);
    speed_amp = 0.5f * (speed_max - speed_min);
    target = speed_mid - speed_amp * FOC_FastCos(phase);
    if (target < speed_min) {
        target = speed_min;
    } else if (target > speed_max) {
        target = speed_max;
    }
    if (g_foc_dyn_speed_reverse != 0U) {
        target = -target;
    }

    g_foc_dyn_speed_elapsed_ms = elapsed_us / 1000U;
    g_foc_dyn_speed_phase_u16 = FOC_Log_ToU16(phase, 65535.0f / FOC_2PI);
    g_foc_dyn_speed_ref_rpm = FOC_Log_ToI16(target, 1.0f);

    return target;
}

static void FOC_DynSpeed_WriteCoreRefWithZeroDir(float rpm, FOC_Dir_e zero_dir)
{
    rpm = FOC_ApplyAppDirectionInvertToRef(rpm);

    if (rpm > s_config.motor.max_speed_rpm) {
        rpm = s_config.motor.max_speed_rpm;
    } else if (rpm < -s_config.motor.max_speed_rpm) {
        rpm = -s_config.motor.max_speed_rpm;
    }
    if (zero_dir > FOC_DIR_CCW) {
        zero_dir = FOC_DIR_CW;
    }

    FOC_HAL_EnterCritical();
    if (rpm > 0.0f) {
        s_ctx.speed_ref = rpm;
        s_ctx.direction = FOC_DIR_CW;
    } else if (rpm < 0.0f) {
        s_ctx.direction = FOC_DIR_CCW;
        s_ctx.speed_ref = -rpm;
    } else {
        s_ctx.speed_ref = 0.0f;
        s_ctx.direction = zero_dir;
    }
    FOC_HAL_ExitCritical();
}

static void FOC_DynSpeed_WriteCoreRef(float rpm)
{
    FOC_DynSpeed_WriteCoreRefWithZeroDir(rpm, s_ctx.direction);
}

static uint8_t FOC_DynSpeed_HandleSetRef(float rpm)
{
    uint32_t now_us;

    g_foc_dyn_core_set_ref_count++;
    g_foc_dyn_speed_last_ext_ref_rpm = FOC_Log_ToI16(rpm, 1.0f);

    if (FOC_DynSpeed_IsStartCommand(rpm) != 0U) {
        if (g_foc_dyn_speed_enable == 0U) {
            now_us = FOC_HAL_GetTimestampUs();
            g_foc_bidir_speed_enable = 0U;
            s_bidir_speed_prev_enable = 0U;
            FOC_BidirZeroTransfer_Reset();
            g_foc_dyn_speed_enable = 1U;
            g_foc_dyn_core_trigger_count++;
            g_foc_dyn_core_trigger_source = 1U;
            s_dyn_speed_prev_enable = 1U;
            FOC_DynSpeed_ResetStats(now_us);
            FOC_DynSpeed_WriteCoreRef(FOC_DynSpeed_CalcRef(now_us));
        }
        return 1U;
    }

    if (g_foc_dyn_speed_enable != 0U) {
        g_foc_dyn_speed_enable = 0U;
        g_foc_dyn_speed_reverse = 0U;
        g_foc_dyn_core_disable_count++;
        s_dyn_speed_prev_enable = 0U;
        g_foc_dyn_speed_reset_stats = 0U;
    }

    if (g_foc_bidir_speed_enable != 0U) {
        g_foc_bidir_speed_enable = 0U;
        g_foc_bidir_speed_step_enable = 0U;
        s_bidir_speed_prev_enable = 0U;
        g_foc_bidir_speed_reset_stats = 0U;
        FOC_BidirZeroTransfer_Reset();
    }

    return 0U;
}

static void FOC_DynSpeed_ServiceRef(void)
{
    uint32_t now_us = FOC_HAL_GetTimestampUs();
    uint8_t livewatch_ref_valid = (speed_ref >= -0.5f) ? 1U : 0U;
    float current_ref = (livewatch_ref_valid != 0U) ? speed_ref : s_ctx.speed_ref;
    float current_fdb = s_ctx.speed_fdb;
    float start_fdb = (float)g_foc_dyn_speed_max_rpm -
                      (float)g_foc_dyn_speed_start_fdb_margin_rpm;

    if ((livewatch_ref_valid == 0U) &&
        (g_foc_dyn_speed_enable != 0U) &&
        (g_foc_dyn_speed_reverse != 0U) &&
        (current_ref > 0.0f)) {
        current_ref = -current_ref;
    }

    g_foc_dyn_core_loop_count++;
    g_foc_dyn_core_state = (uint8_t)s_ctx.state;
    g_foc_dyn_core_seen_ref_rpm = FOC_Log_ToI16(current_ref, 1.0f);
    g_foc_dyn_core_seen_fdb_rpm = FOC_Log_ToI16(current_fdb, 1.0f);
    g_foc_dyn_speed_last_ext_ref_rpm = FOC_Log_ToI16(current_ref, 1.0f);

    if ((livewatch_ref_valid != 0U) && (g_foc_bidir_speed_enable != 0U)) {
        g_foc_bidir_speed_enable = 0U;
        g_foc_bidir_speed_step_enable = 0U;
        g_foc_bidir_speed_reset_stats = 0U;
        s_bidir_speed_prev_enable = 0U;
        FOC_BidirZeroTransfer_Reset();
        if (FOC_DynSpeed_IsStartCommand(current_ref) == 0U) {
            FOC_DynSpeed_WriteCoreRef(current_ref);
        }
    }

    if (g_foc_dyn_speed_reset_stats != 0U) {
        g_foc_dyn_speed_reset_stats = 0U;
        FOC_DynSpeed_ResetStats(now_us);
    }

    if (g_foc_dyn_speed_enable == 0U) {
        if (FOC_DynSpeed_IsStartCommand(current_ref) != 0U) {
            g_foc_bidir_speed_enable = 0U;
            s_bidir_speed_prev_enable = 0U;
            FOC_BidirZeroTransfer_Reset();
            g_foc_dyn_speed_enable = 1U;
            g_foc_dyn_core_trigger_count++;
            g_foc_dyn_core_trigger_source =
                (livewatch_ref_valid != 0U) ? 4U : 2U;
            s_dyn_speed_prev_enable = 1U;
            FOC_DynSpeed_ResetStats(now_us);
        } else if ((livewatch_ref_valid == 0U) &&
                   (g_foc_dyn_speed_start_on_max_fdb != 0U) &&
                   (current_fdb >= start_fdb)) {
            g_foc_bidir_speed_enable = 0U;
            s_bidir_speed_prev_enable = 0U;
            FOC_BidirZeroTransfer_Reset();
            g_foc_dyn_speed_enable = 1U;
            g_foc_dyn_core_trigger_count++;
            g_foc_dyn_core_trigger_source = 3U;
            s_dyn_speed_prev_enable = 1U;
            FOC_DynSpeed_ResetStatsAtMax(now_us);
        } else {
            s_dyn_speed_prev_enable = 0U;
            return;
        }
    } else if ((livewatch_ref_valid != 0U) &&
               (FOC_DynSpeed_IsStartCommand(current_ref) == 0U) &&
               (FOC_DynSpeed_Near(current_ref,
                                  (float)g_foc_dyn_speed_ref_rpm) == 0U)) {
        g_foc_dyn_speed_enable = 0U;
        g_foc_dyn_speed_reverse = 0U;
        g_foc_dyn_core_disable_count++;
        s_dyn_speed_prev_enable = 0U;
        g_foc_dyn_speed_reset_stats = 0U;
        if (livewatch_ref_valid != 0U) {
            FOC_DynSpeed_WriteCoreRef(current_ref);
        }
        return;
    }

    if (s_dyn_speed_prev_enable == 0U) {
        s_dyn_speed_prev_enable = 1U;
        FOC_DynSpeed_ResetStats(now_us);
    }

    FOC_DynSpeed_WriteCoreRef(FOC_DynSpeed_CalcRef(now_us));
}

static int16_t FOC_BidirSpeed_TargetSign(float rpm)
{
    if (rpm > 0.5f) {
        return 1;
    }
    if (rpm < -0.5f) {
        return -1;
    }
    return 0;
}

static FOC_Dir_e FOC_BidirSpeed_DirFromSign(int16_t sign)
{
    return (sign < 0) ? FOC_DIR_CCW : FOC_DIR_CW;
}

static FOC_Dir_e FOC_BidirSpeed_CoreDirFromSign(int16_t sign)
{
    return FOC_ApplyAppDirectionInvertToDir(FOC_BidirSpeed_DirFromSign(sign));
}

static int16_t FOC_BidirZeroTransfer_SignedIqSign(float signed_iq)
{
    if (signed_iq > 0.01f) {
        return 1;
    }
    if (signed_iq < -0.01f) {
        return -1;
    }
    return 0;
}

static float FOC_BidirZeroTransfer_SmoothStep(float x)
{
    x = FOC_CLAMP(x, 0.0f, 1.0f);
    return x * x * (3.0f - (2.0f * x));
}

static float FOC_BidirZeroTransfer_AbsMilliToA(int16_t value_mA)
{
    float value = (float)value_mA * 0.001f;

    if (value < 0.0f) {
        value = -value;
    }
    return value;
}

static float FOC_BidirZeroTransfer_LocalToSignedIq(float local_iq)
{
    return (s_ctx.direction == FOC_DIR_CCW) ? -local_iq : local_iq;
}

static float FOC_BidirZeroTransfer_SignedToLocalIq(float signed_iq)
{
    return (s_ctx.direction == FOC_DIR_CCW) ? -signed_iq : signed_iq;
}

static float FOC_BidirZeroTransfer_LimitSignedIq(float current,
                                                 float target,
                                                 float dt)
{
    float rate =
        (float)g_foc_zero_iq_slew_mA_per_s * 0.001f;
    float step;
    float delta;

    if ((rate <= 0.0f) || (dt <= 0.0f)) {
        return target;
    }

    step = rate * dt;
    delta = target - current;
    if (delta > step) {
        return current + step;
    }
    if (delta < -step) {
        return current - step;
    }
    return target;
}

static void FOC_BidirZeroTransfer_UpdateDebug(uint32_t now_us)
{
    if (FOC_IsAutoTestMotor() == 0U) {
        return;
    }

    g_foc_zero_transfer_state = s_zero_transfer_state;
    g_foc_zero_signed_iq_cmd_mA =
        FOC_Log_ToI16(s_zero_transfer_signed_iq, 1000.0f);
    g_foc_zero_direction_pending = (int8_t)s_zero_transfer_new_sign;

    if (s_zero_transfer_start_us != 0U) {
        uint32_t elapsed_us = now_us - s_zero_transfer_start_us;

        g_foc_zero_transfer_elapsed_ms =
            (uint16_t)((elapsed_us / 1000U) > 65535U
                     ? 65535U
                     : (elapsed_us / 1000U));
    } else {
        g_foc_zero_transfer_elapsed_ms = 0U;
    }

    if (s_ctx.timestamp_prev != 0U) {
        g_foc_zero_edge_elapsed_us = now_us - s_ctx.timestamp_prev;
    } else {
        g_foc_zero_edge_elapsed_us = 0U;
    }
}

static uint8_t FOC_BidirZeroTransfer_Active(void)
{
    if ((g_foc_zero_transfer_enable == 0U) ||
        (g_foc_bidir_speed_enable == 0U) ||
        (s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_IDLE)) {
        return 0U;
    }

    return 1U;
}

static uint8_t FOC_IsAutoTestMotor(void)
{
    return (s_foc_core_active_motor == g_foc_test_motor_id_applied) ? 1U : 0U;
}

static void FOC_BidirZeroTransfer_Reset(void)
{
    s_zero_transfer_state = FOC_ZERO_TRANSFER_STATE_IDLE;
    s_zero_transfer_start_us = 0U;
    s_zero_transfer_last_us = 0U;
    s_zero_transfer_prev_raw = 0.0f;
    s_zero_transfer_signed_iq = 0.0f;
    s_zero_transfer_target_iq = 0.0f;
    s_zero_transfer_old_sign = 0;
    s_zero_transfer_new_sign = 0;
    s_zero_transfer_direction_switched = 0U;

    if (FOC_IsAutoTestMotor() == 0U) {
        return;
    }

    g_foc_zero_transfer_state = FOC_ZERO_TRANSFER_STATE_IDLE;
    g_foc_zero_signed_iq_cmd_mA = 0;
    g_foc_zero_iq_ff_mA = 0;
    g_foc_zero_ctrl_iq_raw_mA = 0;
    g_foc_zero_pid_freeze_active = 0U;
    g_foc_zero_direction_pending = 0;
    g_foc_zero_transfer_elapsed_ms = 0U;
    g_foc_zero_edge_elapsed_us = 0U;
    g_foc_zero_transfer_start_reason = 0U;
    g_foc_zero_transfer_raw_sign = 0;
    g_foc_zero_transfer_prev_sign = 0;
    g_foc_zero_transfer_raw_decreasing = 0U;
    g_foc_zero_transfer_cmd_decreasing = 0U;
    g_foc_zero_transfer_decel_to_zero = 0U;
    g_foc_zero_transfer_raw_abs_rpm = 0U;
    g_foc_zero_transfer_prev_cmd_abs_rpm = 0U;
}

static void FOC_BidirZeroTransfer_Start(uint32_t now_us, int16_t old_sign)
{
    float hold_iq = FOC_BidirZeroTransfer_AbsMilliToA(g_foc_zero_hold_iq_mA);
    float signed_iq = FOC_BidirZeroTransfer_LocalToSignedIq(s_ctx.iq_ref);

    s_zero_transfer_state = FOC_ZERO_TRANSFER_STATE_APPROACH;
    s_zero_transfer_start_us = now_us;
    s_zero_transfer_last_us = now_us;
    s_zero_transfer_old_sign = old_sign;
    s_zero_transfer_new_sign = (old_sign < 0) ? 1 : -1;
    s_zero_transfer_direction_switched = 0U;

    if ((FOC_BidirZeroTransfer_SignedIqSign(signed_iq) != old_sign) ||
        (FOC_FABS(signed_iq) < hold_iq)) {
        signed_iq = (old_sign < 0) ? -hold_iq : hold_iq;
    }
    s_zero_transfer_signed_iq = signed_iq;
    s_zero_transfer_target_iq = signed_iq;

    if ((FOC_IsAutoTestMotor() != 0U) &&
        (g_foc_zero_transfer_count < 0xFFFFFFFFU)) {
        g_foc_zero_transfer_count++;
    }
}

static float FOC_BidirZeroTransfer_ServiceRef(float target,
                                              float raw_target,
                                              uint32_t now_us,
                                              uint8_t raw_decel_to_zero,
                                              FOC_Dir_e *zero_dir)
{
    float raw_abs = FOC_FABS(raw_target);
    float prev_abs = FOC_FABS(s_zero_transfer_prev_raw);
    float enter_rpm = (float)g_foc_zero_transfer_enter_rpm;
    float exit_rpm = (float)g_foc_zero_transfer_exit_rpm;
    float prev_cmd_abs = FOC_FABS(s_ctx.speed_ref);
    int16_t raw_sign = FOC_BidirSpeed_TargetSign(raw_target);
    int16_t prev_sign = FOC_BidirSpeed_TargetSign(s_zero_transfer_prev_raw);
    uint8_t decreasing = ((prev_abs - raw_abs) > 0.5f) ? 1U : 0U;
    uint8_t cmd_decreasing =
        ((prev_cmd_abs - raw_abs) > 0.5f) ? 1U : 0U;
    int16_t start_sign = raw_sign;
    uint8_t start_reason = 0U;

    if (zero_dir == NULL) {
        return target;
    }

    if ((g_foc_zero_transfer_enable == 0U) ||
        (g_foc_bidir_speed_enable == 0U)) {
        FOC_BidirZeroTransfer_Reset();
        s_zero_transfer_prev_raw = raw_target;
        return target;
    }

    if (enter_rpm < 1.0f) {
        enter_rpm = 1.0f;
    }
    if (exit_rpm < 1.0f) {
        exit_rpm = 1.0f;
    }
    if (exit_rpm > enter_rpm) {
        exit_rpm = enter_rpm;
    }

    if (start_sign == 0) {
        start_sign = (s_ctx.direction == FOC_DIR_CCW) ? -1 : 1;
    }
    if (FOC_IsAutoTestMotor() != 0U) {
        g_foc_zero_transfer_raw_sign = (int8_t)raw_sign;
        g_foc_zero_transfer_prev_sign = (int8_t)prev_sign;
        g_foc_zero_transfer_raw_decreasing = decreasing;
        g_foc_zero_transfer_cmd_decreasing = cmd_decreasing;
        g_foc_zero_transfer_decel_to_zero = raw_decel_to_zero;
        g_foc_zero_transfer_raw_abs_rpm = FOC_Log_ToU16(raw_abs, 1.0f);
        g_foc_zero_transfer_prev_cmd_abs_rpm =
            FOC_Log_ToU16(prev_cmd_abs, 1.0f);
    }

    if ((raw_decel_to_zero != 0U) &&
        (raw_sign != 0)) {
        start_reason = 4U;
    } else if ((raw_decel_to_zero != 0U) &&
               (raw_sign == 0)) {
        start_reason = 5U;
    } else if ((raw_sign != 0) &&
        (decreasing != 0U) &&
        ((prev_sign == raw_sign) || (prev_sign == 0))) {
        start_reason = 1U;
    } else if ((raw_sign != 0) &&
               (cmd_decreasing != 0U)) {
        start_reason = 2U;
    } else if ((raw_sign == 0) &&
               (cmd_decreasing != 0U)) {
        start_reason = 3U;
    }

    if ((s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_IDLE) &&
        (start_reason != 0U) &&
        (start_sign != 0) &&
        (raw_abs <= enter_rpm)) {
        FOC_BidirZeroTransfer_Start(now_us, start_sign);
        if (FOC_IsAutoTestMotor() != 0U) {
            g_foc_zero_transfer_start_reason = start_reason;
        }
    }

    if (s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_APPROACH) {
        *zero_dir = FOC_BidirSpeed_CoreDirFromSign(s_zero_transfer_old_sign);
        target = (s_zero_transfer_old_sign < 0) ? -raw_abs : raw_abs;

        if ((raw_sign == s_zero_transfer_new_sign) ||
            (raw_abs <= exit_rpm)) {
            s_zero_transfer_state = FOC_ZERO_TRANSFER_STATE_TRANSFER;
            s_zero_transfer_start_us = now_us;
            s_zero_transfer_last_us = now_us;
        }
    }

    if (s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_TRANSFER) {
        uint32_t transfer_us =
            (uint32_t)g_foc_zero_transfer_ms * 1000U;
        uint32_t elapsed_us = now_us - s_zero_transfer_start_us;
        float hold_iq =
            FOC_BidirZeroTransfer_AbsMilliToA(g_foc_zero_hold_iq_mA);
        float breakaway_iq =
            FOC_BidirZeroTransfer_AbsMilliToA(g_foc_zero_breakaway_iq_mA);
        float progress;
        float ease;
        float start_iq =
            (s_zero_transfer_old_sign < 0) ? -hold_iq : hold_iq;
        float end_iq =
            (s_zero_transfer_new_sign < 0) ? -breakaway_iq : breakaway_iq;

        if (transfer_us == 0U) {
            transfer_us = 1U;
        }
        progress = FOC_CLAMP((float)elapsed_us / (float)transfer_us,
                             0.0f, 1.0f);
        ease = FOC_BidirZeroTransfer_SmoothStep(progress);
        s_zero_transfer_target_iq =
            start_iq + ((end_iq - start_iq) * ease);

        if ((s_zero_transfer_direction_switched == 0U) &&
            (FOC_BidirZeroTransfer_SignedIqSign(s_zero_transfer_signed_iq) ==
             s_zero_transfer_new_sign)) {
            s_zero_transfer_direction_switched = 1U;
        }

        *zero_dir =
            (s_zero_transfer_direction_switched != 0U)
          ? FOC_BidirSpeed_CoreDirFromSign(s_zero_transfer_new_sign)
          : FOC_BidirSpeed_CoreDirFromSign(s_zero_transfer_old_sign);
        target = 0.0f;

        if ((progress >= 1.0f) &&
            (s_zero_transfer_direction_switched != 0U)) {
            s_zero_transfer_state = FOC_ZERO_TRANSFER_STATE_RELAUNCH;
            s_zero_transfer_start_us = now_us;
        }
    }

    if (s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_RELAUNCH) {
        uint32_t relaunch_us =
            (uint32_t)g_foc_zero_relaunch_ms * 1000U;
        uint32_t relaunch_elapsed_us = now_us - s_zero_transfer_start_us;
        uint32_t edge_elapsed_us =
            (s_ctx.timestamp_prev != 0U) ? (now_us - s_ctx.timestamp_prev)
                                         : 0xFFFFFFFFU;
        uint32_t edge_max_us = g_foc_zero_relaunch_edge_max_us;
        uint8_t edge_recent = 1U;
        uint8_t relaunch_ready = 0U;
        float relaunch_abs = raw_abs;

        if (relaunch_us == 0U) {
            relaunch_us = 1U;
        }
        if (edge_max_us != 0U) {
            edge_recent = (edge_elapsed_us <= edge_max_us) ? 1U : 0U;
        }
        *zero_dir = FOC_BidirSpeed_CoreDirFromSign(s_zero_transfer_new_sign);
        if (relaunch_abs < exit_rpm) {
            relaunch_abs = exit_rpm;
        }
        target = (s_zero_transfer_new_sign < 0) ? -relaunch_abs : relaunch_abs;

        if ((raw_sign == s_zero_transfer_new_sign) &&
            (relaunch_elapsed_us >= relaunch_us)) {
            if (raw_abs >= exit_rpm) {
                relaunch_ready = 1U;
            } else if ((edge_recent != 0U) &&
                       (FOC_FABS(s_ctx.speed_fdb) >= exit_rpm)) {
                relaunch_ready = 1U;
            }
            if (relaunch_ready != 0U) {
                FOC_BidirZeroTransfer_Reset();
            }
        }
    }

    if (s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_IDLE) {
        g_foc_zero_pid_freeze_active = 0U;
        g_foc_zero_direction_pending = 0;
    }

    FOC_BidirZeroTransfer_UpdateDebug(now_us);
    s_zero_transfer_prev_raw = raw_target;
    return target;
}

static uint8_t FOC_BidirZeroTransfer_PidFrozen(void)
{
    if ((g_foc_zero_transfer_enable == 0U) ||
        (g_foc_bidir_speed_enable == 0U)) {
        return 0U;
    }

    return (s_zero_transfer_state >= FOC_ZERO_TRANSFER_STATE_TRANSFER)
         ? 1U
         : 0U;
}

static float FOC_BidirZeroTransfer_ApplyIq(float iq_ref,
                                           float speed_dt)
{
    float ctrl_signed = FOC_BidirZeroTransfer_LocalToSignedIq(iq_ref);
    float desired_signed = ctrl_signed;
    float hold_iq = FOC_BidirZeroTransfer_AbsMilliToA(g_foc_zero_hold_iq_mA);
    float breakaway_iq =
        FOC_BidirZeroTransfer_AbsMilliToA(g_foc_zero_breakaway_iq_mA);

    g_foc_zero_ctrl_iq_raw_mA = FOC_Log_ToI16(ctrl_signed, 1000.0f);
    g_foc_zero_pid_freeze_active = FOC_BidirZeroTransfer_PidFrozen();

    if ((g_foc_zero_transfer_enable == 0U) ||
        (s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_IDLE)) {
        g_foc_zero_iq_ff_mA = 0;
        return iq_ref;
    }

    if (s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_APPROACH) {
        desired_signed = (s_zero_transfer_old_sign < 0) ? -hold_iq : hold_iq;
        if ((FOC_BidirZeroTransfer_SignedIqSign(ctrl_signed) ==
             s_zero_transfer_old_sign) &&
            (FOC_FABS(ctrl_signed) > hold_iq)) {
            desired_signed = ctrl_signed;
        }
    } else if (s_zero_transfer_state == FOC_ZERO_TRANSFER_STATE_TRANSFER) {
        desired_signed = s_zero_transfer_target_iq;
    } else {
        desired_signed =
            (s_zero_transfer_new_sign < 0) ? -breakaway_iq : breakaway_iq;
        if ((FOC_BidirZeroTransfer_SignedIqSign(ctrl_signed) ==
             s_zero_transfer_new_sign) &&
            (FOC_FABS(ctrl_signed) > breakaway_iq)) {
            desired_signed = ctrl_signed;
        }
    }

    s_zero_transfer_signed_iq =
        FOC_BidirZeroTransfer_LimitSignedIq(s_zero_transfer_signed_iq,
                                            desired_signed,
                                            speed_dt);
    g_foc_zero_signed_iq_cmd_mA =
        FOC_Log_ToI16(s_zero_transfer_signed_iq, 1000.0f);
    g_foc_zero_iq_ff_mA =
        FOC_Log_ToI16(s_zero_transfer_signed_iq - ctrl_signed, 1000.0f);

    return FOC_BidirZeroTransfer_SignedToLocalIq(s_zero_transfer_signed_iq);
}

static float FOC_BidirSpeed_LimitTargetStep(float current,
                                            float target,
                                            float slew_rpm_per_s,
                                            uint32_t dt_us)
{
    float max_delta;
    float delta;

    if (slew_rpm_per_s < 1.0f) {
        slew_rpm_per_s = 1.0f;
    }
    if (dt_us == 0U) {
        dt_us = FOC_CONTROL_PERIOD_US;
    } else if (dt_us > FOC_CONTROL_PID_DT_MAX_US) {
        dt_us = FOC_CONTROL_PID_DT_MAX_US;
    }

    max_delta = slew_rpm_per_s * ((float)dt_us * 1.0e-6f);
    delta = target - current;
    if (delta > max_delta) {
        return current + max_delta;
    }
    if (delta < -max_delta) {
        return current - max_delta;
    }

    return target;
}

static void FOC_BidirSpeed_ResetZeroCross(void)
{
    s_bidir_zero_state = FOC_BIDIR_ZERO_STATE_IDLE;
    s_bidir_zero_start_us = 0U;
    s_bidir_zero_hold_start_us = 0U;
    s_bidir_zero_below_start_us = 0U;
    s_bidir_zero_last_us = 0U;
    s_bidir_zero_ref_rpm = 0.0f;
    s_bidir_zero_command_sign = 0;
    s_bidir_zero_pending_sign = 0;
    s_bidir_zero_hold_dir = s_ctx.direction;
    g_foc_bidir_zero_cross_state = FOC_BIDIR_ZERO_STATE_IDLE;
    g_foc_bidir_zero_cross_elapsed_ms = 0U;
    g_foc_bidir_zero_below_elapsed_ms = 0U;
    g_foc_bidir_zero_approach_active = 0U;
    g_foc_bidir_zero_tail_active = 0U;
    g_foc_bidir_zero_tail_fdb_catch_count = 0U;
    g_foc_bidir_zero_tail_drive_assist_count = 0U;
    g_foc_bidir_zero_ref_rpm = 0;
}

static float FOC_BidirSpeed_ApplyZeroCross(float target,
                                           float raw_target,
                                           uint32_t now_us,
                                           FOC_Dir_e *zero_dir)
{
    int16_t raw_sign = FOC_BidirSpeed_TargetSign(raw_target);
    uint32_t elapsed_us = 0U;
    uint32_t hold_us;
    uint32_t timeout_us;
    uint32_t confirm_us;
    uint32_t below_elapsed_us = 0U;
    uint16_t zero_speed = g_foc_bidir_zero_speed_rpm;
    uint16_t tail_start = g_foc_bidir_zero_tail_start_rpm;

    if (zero_dir == NULL) {
        return target;
    }
    *zero_dir = s_ctx.direction;

    if (g_foc_bidir_zero_cross_enable == 0U) {
        FOC_BidirSpeed_ResetZeroCross();
        if (raw_sign != 0) {
            s_bidir_zero_command_sign = raw_sign;
        }
        return target;
    }

    if (s_bidir_zero_command_sign == 0) {
        if (raw_sign != 0) {
            s_bidir_zero_command_sign = raw_sign;
        }
        g_foc_bidir_zero_cross_state = s_bidir_zero_state;
        return target;
    }

    if ((s_bidir_zero_state == FOC_BIDIR_ZERO_STATE_IDLE) &&
        (raw_sign != 0) &&
        (raw_sign != s_bidir_zero_command_sign)) {
        float decel_start_ref = FOC_FABS(target);

        s_bidir_zero_state = FOC_BIDIR_ZERO_STATE_DECEL;
        s_bidir_zero_start_us = now_us;
        s_bidir_zero_hold_start_us = 0U;
        s_bidir_zero_below_start_us = 0U;
        s_bidir_zero_last_us = now_us;
        s_bidir_zero_ref_rpm = decel_start_ref;
        s_bidir_zero_pending_sign = raw_sign;
        s_bidir_zero_hold_dir =
            FOC_BidirSpeed_CoreDirFromSign(s_bidir_zero_command_sign);
        if (g_foc_bidir_zero_cross_count < 0xFFFFFFFFU) {
            g_foc_bidir_zero_cross_count++;
        }
    }

    if (s_bidir_zero_state == FOC_BIDIR_ZERO_STATE_DECEL) {
        if (raw_sign != 0) {
            s_bidir_zero_pending_sign = raw_sign;
        }
        timeout_us = (uint32_t)g_foc_bidir_zero_timeout_ms * 1000U;
        confirm_us = (uint32_t)g_foc_bidir_zero_confirm_ms * 1000U;
        elapsed_us = now_us - s_bidir_zero_start_us;
        *zero_dir = s_bidir_zero_hold_dir;
        g_foc_bidir_zero_tail_active = 0U;
        if (tail_start < zero_speed) {
            tail_start = zero_speed;
        }
        target = (s_bidir_zero_command_sign < 0)
               ? -s_bidir_zero_ref_rpm
               : s_bidir_zero_ref_rpm;
        if (s_bidir_zero_ref_rpm <= (float)tail_start) {
            float tail_slew =
                (float)g_foc_bidir_zero_tail_slew_rpm_per_s;

            if (tail_slew >= 1.0f) {
                g_foc_bidir_zero_tail_active = 1U;
                target = FOC_BidirSpeed_LimitTargetStep(
                    target, 0.0f, tail_slew,
                    now_us - s_bidir_zero_last_us);
            } else {
                target = FOC_BidirSpeed_LimitTargetStep(
                    target, 0.0f,
                    (float)g_foc_bidir_zero_approach_slew_rpm_per_s,
                    now_us - s_bidir_zero_last_us);
            }
        } else {
            target = FOC_BidirSpeed_LimitTargetStep(
                target, 0.0f,
                (float)g_foc_bidir_zero_approach_slew_rpm_per_s,
                now_us - s_bidir_zero_last_us);
        }
        if (FOC_FABS(target) < 0.5f) {
            target = 0.0f;
        }
        s_bidir_zero_last_us = now_us;
        s_bidir_zero_ref_rpm = FOC_FABS(target);
        s_bidir_speed_limited_ref_rpm = target;
        g_foc_bidir_zero_approach_active = 1U;
        g_foc_bidir_zero_ref_rpm = FOC_Log_ToI16(target, 1.0f);

        if ((s_bidir_zero_ref_rpm <= (float)zero_speed) &&
            (FOC_FABS(s_ctx.speed_fdb) <= (float)zero_speed)) {
            if (s_bidir_zero_below_start_us == 0U) {
                s_bidir_zero_below_start_us = now_us;
            }
            below_elapsed_us = now_us - s_bidir_zero_below_start_us;
        } else {
            s_bidir_zero_below_start_us = 0U;
            below_elapsed_us = 0U;
        }

        if ((s_bidir_zero_ref_rpm <= (float)zero_speed) &&
            ((((s_bidir_zero_below_start_us != 0U) &&
               (below_elapsed_us >= confirm_us)) ||
              ((timeout_us > 0U) && (elapsed_us >= timeout_us))))) {
            s_bidir_zero_state = FOC_BIDIR_ZERO_STATE_HOLD;
            s_bidir_zero_hold_start_us = now_us;
        }
    }

    if (s_bidir_zero_state == FOC_BIDIR_ZERO_STATE_HOLD) {
        hold_us = (uint32_t)g_foc_bidir_zero_hold_ms * 1000U;
        elapsed_us = now_us - s_bidir_zero_start_us;
        *zero_dir = s_bidir_zero_hold_dir;
        target = 0.0f;
        s_bidir_speed_limited_ref_rpm = 0.0f;
        s_bidir_zero_ref_rpm = 0.0f;
        g_foc_bidir_zero_approach_active = 1U;
        g_foc_bidir_zero_tail_active = 0U;
        g_foc_bidir_zero_ref_rpm = 0;

        if ((now_us - s_bidir_zero_hold_start_us) >= hold_us) {
            if (s_bidir_zero_pending_sign != 0) {
                s_bidir_zero_command_sign = s_bidir_zero_pending_sign;
            }
            s_bidir_zero_state = FOC_BIDIR_ZERO_STATE_IDLE;
            s_bidir_zero_last_us = 0U;
            *zero_dir = FOC_BidirSpeed_CoreDirFromSign(s_bidir_zero_command_sign);
        }
    }

    if (s_bidir_zero_state == FOC_BIDIR_ZERO_STATE_IDLE) {
        elapsed_us = 0U;
        below_elapsed_us = 0U;
    }
    g_foc_bidir_zero_cross_state = s_bidir_zero_state;
    g_foc_bidir_zero_cross_elapsed_ms =
        (uint16_t)((elapsed_us / 1000U) > 65535U
                 ? 65535U
                 : (elapsed_us / 1000U));
    g_foc_bidir_zero_below_elapsed_ms =
        (uint16_t)((below_elapsed_us / 1000U) > 65535U
                 ? 65535U
                 : (below_elapsed_us / 1000U));

    return target;
}

static void FOC_BidirSpeed_ServiceRef(void)
{
    uint32_t now_us = FOC_HAL_GetTimestampUs();
    uint32_t period_ms;
    uint32_t period_us;
    uint32_t elapsed_us;
    uint32_t phase_us;
    uint32_t dt_us;
    float phase;
    float raw_target;
    float target;
    FOC_Dir_e zero_dir = s_ctx.direction;
    uint8_t old_zero_cross_enabled;
    uint8_t raw_decel_to_zero = 0U;

    if (g_foc_bidir_speed_reset_stats != 0U) {
        g_foc_bidir_speed_reset_stats = 0U;
        s_bidir_speed_prev_enable = 0U;
        FOC_BidirSpeed_ResetZeroCross();
        FOC_BidirZeroTransfer_Reset();
    }

    if (g_foc_bidir_speed_enable == 0U) {
        s_bidir_speed_prev_enable = 0U;
        FOC_BidirSpeed_ResetZeroCross();
        FOC_BidirZeroTransfer_Reset();
        return;
    }

    if (s_bidir_speed_prev_enable == 0U) {
        s_bidir_speed_prev_enable = 1U;
        s_bidir_speed_start_us = now_us;
        s_bidir_speed_last_us = now_us;
        s_bidir_speed_limited_ref_rpm = 0.0f;
        FOC_BidirSpeed_ResetZeroCross();
        FOC_BidirZeroTransfer_Reset();
        FOC_DynSpeed_ResetStats(now_us);
    }

    period_ms = g_foc_bidir_speed_period_ms;
    if (period_ms < 1000U) {
        period_ms = 1000U;
    }

    period_us = period_ms * 1000U;
    elapsed_us = now_us - s_bidir_speed_start_us;
    phase_us = (period_us > 0U) ? (elapsed_us % period_us) : 0U;
    phase = ((float)phase_us / (float)period_us) * FOC_2PI;
    if (g_foc_bidir_speed_step_enable == 1U) {
        raw_target = (phase_us < (period_us / 2U))
                   ? (float)g_foc_bidir_speed_max_rpm
                   : -(float)g_foc_bidir_speed_max_rpm;
    } else if (g_foc_bidir_speed_step_enable == 2U) {
        uint32_t quarter_us = period_us / 4U;
        uint32_t segment;
        uint32_t segment_us;
        float u;
        float ease;

        if (quarter_us == 0U) {
            quarter_us = 1U;
        }
        segment = phase_us / quarter_us;
        if (segment > 3U) {
            segment = 3U;
        }
        if ((segment == 1U) || (segment == 3U)) {
            raw_decel_to_zero = 1U;
        }
        segment_us = phase_us - (segment * quarter_us);
        u = (float)segment_us / (float)quarter_us;
        ease = 0.5f * (1.0f - FOC_FastCos(FOC_PI * u));

        if (segment == 0U) {
            raw_target = (float)g_foc_bidir_speed_max_rpm * ease;
        } else if (segment == 1U) {
            raw_target = (float)g_foc_bidir_speed_max_rpm *
                         (1.0f - ease);
        } else if (segment == 2U) {
            raw_target = -(float)g_foc_bidir_speed_max_rpm * ease;
        } else {
            raw_target = -(float)g_foc_bidir_speed_max_rpm *
                         (1.0f - ease);
        }
    } else {
        raw_target = (float)g_foc_bidir_speed_max_rpm * FOC_FastSin(phase);
    }

    if (raw_target > s_config.motor.max_speed_rpm) {
        raw_target = s_config.motor.max_speed_rpm;
    } else if (raw_target < -s_config.motor.max_speed_rpm) {
        raw_target = -s_config.motor.max_speed_rpm;
    }

    target = raw_target;
    dt_us = now_us - s_bidir_speed_last_us;
    old_zero_cross_enabled =
        ((g_foc_zero_transfer_enable == 0U) &&
         (g_foc_bidir_zero_cross_enable != 0U)) ? 1U : 0U;
    g_foc_bidir_zero_approach_active = 0U;
    if ((g_foc_bidir_speed_slew_enable != 0U) ||
        (old_zero_cross_enabled != 0U)) {
        float slew = (float)g_foc_bidir_speed_slew_rpm_per_s;
        int16_t raw_sign = FOC_BidirSpeed_TargetSign(raw_target);
        int16_t limited_sign =
            FOC_BidirSpeed_TargetSign(s_bidir_speed_limited_ref_rpm);

        if ((old_zero_cross_enabled != 0U) &&
            (limited_sign != 0) &&
            ((raw_sign == limited_sign) || (raw_sign == 0)) &&
            (FOC_FABS(raw_target) < FOC_FABS(s_bidir_speed_limited_ref_rpm)) &&
            (FOC_FABS(raw_target) <=
             (float)g_foc_bidir_zero_approach_start_rpm)) {
            float approach_slew =
                (float)g_foc_bidir_zero_approach_slew_rpm_per_s;

            if ((approach_slew >= 1.0f) && (approach_slew < slew)) {
                slew = approach_slew;
            }
            g_foc_bidir_zero_approach_active = 1U;
        }
        target = FOC_BidirSpeed_LimitTargetStep(
            s_bidir_speed_limited_ref_rpm, raw_target, slew, dt_us);
        s_bidir_speed_limited_ref_rpm = target;
    } else {
        s_bidir_speed_limited_ref_rpm = target;
    }
    s_bidir_speed_last_us = now_us;
    target = FOC_BidirZeroTransfer_ServiceRef(target, raw_target, now_us,
                                              raw_decel_to_zero,
                                              &zero_dir);
    s_bidir_speed_limited_ref_rpm = target;
    if (old_zero_cross_enabled != 0U) {
        target = FOC_BidirSpeed_ApplyZeroCross(target, raw_target, now_us,
                                               &zero_dir);
    } else {
        FOC_BidirSpeed_ResetZeroCross();
    }
    if ((g_foc_bidir_zero_approach_active == 0U) &&
        (s_bidir_zero_state == FOC_BIDIR_ZERO_STATE_IDLE)) {
        g_foc_bidir_zero_ref_rpm = 0;
    } else {
        g_foc_bidir_zero_ref_rpm = FOC_Log_ToI16(target, 1.0f);
    }

    g_foc_bidir_speed_elapsed_ms = elapsed_us / 1000U;
    g_foc_bidir_speed_phase_u16 = FOC_Log_ToU16(phase, 65535.0f / FOC_2PI);
    g_foc_bidir_speed_raw_ref_rpm = FOC_Log_ToI16(raw_target, 1.0f);
    g_foc_bidir_speed_ref_rpm = FOC_Log_ToI16(target, 1.0f);

    g_foc_dyn_speed_elapsed_ms = g_foc_bidir_speed_elapsed_ms;
    g_foc_dyn_speed_phase_u16 = g_foc_bidir_speed_phase_u16;
    g_foc_dyn_speed_ref_rpm = g_foc_bidir_speed_ref_rpm;
    g_foc_dyn_speed_last_ext_ref_rpm = g_foc_bidir_speed_ref_rpm;

    FOC_DynSpeed_WriteCoreRefWithZeroDir(target, zero_dir);
}

static void FOC_DynSpeed_RecordLog(uint32_t now_us, int16_t err_rpm)
{
    uint16_t idx;
    uint32_t decim_us = (uint32_t)g_foc_dyn_log_decim_ms * 1000U;
    float signed_speed_fdb = s_ctx.speed_fdb;
    float signed_speed_ctrl_fdb = s_ctx.speed_ctrl_fdb;

    if ((g_foc_dyn_speed_ref_rpm < 0) &&
        (s_ctx.direction == FOC_DIR_CCW)) {
        signed_speed_fdb = -signed_speed_fdb;
        signed_speed_ctrl_fdb = -signed_speed_ctrl_fdb;
    }

    if (g_foc_dyn_log_stop != 0U) {
        return;
    }
    if (decim_us < 1000U) {
        decim_us = 1000U;
    }
    if ((s_dyn_log_last_us != 0U) &&
        ((now_us - s_dyn_log_last_us) < decim_us)) {
        return;
    }

    idx = g_foc_dyn_log_idx;
    if (idx >= FOC_DYN_SPEED_LOG_SIZE) {
        g_foc_dyn_log_stop = 1U;
        return;
    }

    s_dyn_log_last_us = now_us;
    g_foc_dyn_log_t_ms[idx] = g_foc_dyn_speed_elapsed_ms;
    g_foc_dyn_log_ref_rpm[idx] = g_foc_dyn_speed_ref_rpm;
    g_foc_dyn_log_fdb_rpm[idx] = FOC_Log_ToI16(signed_speed_fdb, 1.0f);
    g_foc_dyn_log_ctrl_fdb_rpm[idx] =
        FOC_Log_ToI16(signed_speed_ctrl_fdb, 1.0f);
    g_foc_dyn_log_err_rpm[idx] = err_rpm;
    g_foc_dyn_log_iq_ref_mA[idx] = FOC_Log_ToI16(FOC_ControlIqRef(), 1000.0f);
    g_foc_dyn_log_iq_mA[idx] = FOC_Log_ToI16(s_ctx.i_dq.q, 1000.0f);
    g_foc_dyn_log_current_peak_mA[idx] = FOC_Log_ToU16(s_ctx.current_peak, 1000.0f);
    g_foc_dyn_log_fault[idx] = (uint16_t)s_ctx.fault;

    idx++;
    g_foc_dyn_log_idx = idx;
    if (idx >= FOC_DYN_SPEED_LOG_SIZE) {
        g_foc_dyn_log_stop = 1U;
    }
}

static void FOC_DynSpeed_ServiceMetrics(void)
{
    uint32_t now_us;
    float err;
    float abs_err;
    float signed_speed_fdb = s_ctx.speed_fdb;
    float signed_speed_ctrl_fdb = s_ctx.speed_ctrl_fdb;
    int16_t err_rpm;

    if ((g_foc_dyn_speed_enable == 0U) &&
        (g_foc_bidir_speed_enable == 0U)) {
        return;
    }

    now_us = FOC_HAL_GetTimestampUs();
    if ((g_foc_dyn_speed_ref_rpm < 0) &&
        (s_ctx.direction == FOC_DIR_CCW)) {
        signed_speed_fdb = -signed_speed_fdb;
        signed_speed_ctrl_fdb = -signed_speed_ctrl_fdb;
    }

    err = (float)g_foc_dyn_speed_ref_rpm - signed_speed_fdb;
    abs_err = FOC_FABS(err);
    err_rpm = FOC_Log_ToI16(err, 1.0f);

    g_foc_dyn_speed_fdb_rpm = FOC_Log_ToI16(signed_speed_fdb, 1.0f);
    g_foc_dyn_speed_ctrl_fdb_rpm =
        FOC_Log_ToI16(signed_speed_ctrl_fdb, 1.0f);
    g_foc_dyn_speed_err_rpm = err_rpm;
    g_foc_dyn_speed_abs_err_rpm = FOC_Log_ToU16(abs_err, 1.0f);
    if (g_foc_dyn_speed_abs_err_rpm > g_foc_dyn_speed_max_abs_err_rpm) {
        g_foc_dyn_speed_max_abs_err_rpm = g_foc_dyn_speed_abs_err_rpm;
    }

    s_dyn_abs_err_avg_rpm =
        0.001f * abs_err + 0.999f * s_dyn_abs_err_avg_rpm;
    g_foc_dyn_speed_abs_err_avg_rpm =
        FOC_Log_ToU16(s_dyn_abs_err_avg_rpm, 1.0f);

    g_foc_dyn_speed_iq_ref_mA = FOC_Log_ToI16(FOC_ControlIqRef(), 1000.0f);
    g_foc_dyn_speed_id_mA = FOC_Log_ToI16(s_ctx.i_dq.d, 1000.0f);
    g_foc_dyn_speed_iq_mA = FOC_Log_ToI16(s_ctx.i_dq.q, 1000.0f);
    g_foc_dyn_speed_current_peak_mA =
        FOC_Log_ToU16(s_ctx.current_peak, 1000.0f);
    g_foc_dyn_speed_sample_count++;

    FOC_DynSpeed_RecordLog(now_us, err_rpm);
}

 static void FOC_ResetCurrentAngleTrim(void)
 {
     s_current_angle_trim_rad = 0.0f;
     g_foc_current_angle_trim_mrad = 0;
 }

static float FOC_ApplyCurrentAngleTrim(float theta_ctrl)
{
     float offset_rad = 0.0f;
     int16_t offset_mrad;

     if (s_ctx.direction == FOC_DIR_CCW) {
         offset_mrad = (s_foc_core_active_motor == 1U) ?
             g_foc_motor1_ccw_angle_offset_mrad :
             g_foc_ccw_angle_offset_mrad;
     } else {
         offset_mrad = (s_foc_core_active_motor == 1U) ?
             g_foc_motor1_cw_angle_offset_mrad :
             g_foc_cw_angle_offset_mrad;
     }
     offset_rad += (float)offset_mrad * 0.001f;

#if FOC_CURRENT_ANGLE_TRIM_ENABLE
     offset_rad += s_current_angle_trim_rad;
#endif

     g_foc_control_angle_offset_mrad = FOC_Log_ToI16(offset_rad, 1000.0f);
     theta_ctrl += offset_rad;
     return FOC_NormalizeAngle(theta_ctrl);
 }

static void FOC_UpdateCurrentAngleTrim(float dt)
{
#if FOC_CURRENT_ANGLE_TRIM_ENABLE
     float speed_ref_abs = FOC_FABS(s_ctx.speed_ref);
     float speed_err = FOC_FABS(s_ctx.speed_ref - s_ctx.speed_fdb);
     float iq_abs = FOC_FABS(s_ctx.i_dq.q);

     if ((dt > 0.0f) &&
         (speed_ref_abs >= FOC_CURRENT_ANGLE_TRIM_MIN_SPEED_RPM) &&
         (speed_err <= FOC_CURRENT_ANGLE_TRIM_MAX_SPEED_ERR_RPM) &&
         (iq_abs >= FOC_CURRENT_ANGLE_TRIM_MIN_IQ_A)) {
         float id_norm = s_ctx.i_dq.d / iq_abs;

         id_norm = FOC_CLAMP(id_norm, -1.0f, 1.0f);
         s_current_angle_trim_rad -=
             FOC_CURRENT_ANGLE_TRIM_GAIN * dt * id_norm;
         s_current_angle_trim_rad = FOC_CLAMP(s_current_angle_trim_rad,
                                              -FOC_CURRENT_ANGLE_TRIM_MAX_RAD,
                                               FOC_CURRENT_ANGLE_TRIM_MAX_RAD);
     } else {
         s_current_angle_trim_rad *= FOC_CURRENT_ANGLE_TRIM_DECAY;
         if (FOC_FABS(s_current_angle_trim_rad) < 0.0005f) {
             s_current_angle_trim_rad = 0.0f;
         }
     }

     g_foc_current_angle_trim_mrad =
         FOC_Log_ToI16(s_current_angle_trim_rad, 1000.0f);
#else
     (void)dt;
     FOC_ResetCurrentAngleTrim();
#endif
}

static void FOC_DecaySpeedControlFeedback(float target_fdb)
{
    float decay_rate = (float)g_foc_speed_ctrl_fdb_no_edge_decay_rpm_per_s;
    float max_lead = (float)g_foc_speed_ctrl_fdb_max_lead_rpm;
    float decay_step;
    float delta;

    if (target_fdb < 0.0f) {
        target_fdb = 0.0f;
    }
    if (s_ctx.speed_ctrl_fdb <= target_fdb) {
        s_ctx.speed_ctrl_fdb = target_fdb;
        return;
    }

    if (decay_rate < 1.0f) {
        decay_rate = 1.0f;
    }
    decay_step =
        decay_rate * ((float)s_foc_control_period_us * 1.0e-6f);
    delta = s_ctx.speed_ctrl_fdb - target_fdb;

    if (decay_step >= delta) {
        s_ctx.speed_ctrl_fdb = target_fdb;
    } else {
        s_ctx.speed_ctrl_fdb -= decay_step;
    }
    if (s_ctx.speed_ctrl_fdb < 0.0f) {
        s_ctx.speed_ctrl_fdb = 0.0f;
    }
    if ((max_lead > 0.0f) &&
        (s_ctx.speed_ctrl_fdb > (target_fdb + max_lead))) {
        s_ctx.speed_ctrl_fdb = target_fdb + max_lead;
    }
    if (g_foc_speed_ctrl_fdb_no_edge_decay_count < 0xFFFFFFFFU) {
        g_foc_speed_ctrl_fdb_no_edge_decay_count++;
    }
}

static void FOC_ResetSpeedDropFaultMonitor(void)
{
    g_foc_speed_drop_fault_count = 0U;
    g_foc_speed_drop_fault_ref_rpm = 0;
    g_foc_speed_drop_fault_fdb_rpm = 0;
    g_foc_speed_drop_fault_ctrl_fdb_rpm = 0;
    g_foc_speed_drop_fault_err_last_rpm = 0;
    g_foc_speed_drop_fault_iq_ref_mA = 0;
    g_foc_speed_drop_fault_sector_no_change_count = 0U;
    g_foc_speed_drop_fault_edge_elapsed_us = 0U;
    g_foc_speed_drop_fault_direction = 0U;
    g_foc_speed_drop_fault_bidir_no_edge_skip = 0U;
    g_foc_speed_drop_fault_bidir_no_edge_holdoff_count = 0U;
    s_speed_drop_bidir_no_edge_holdoff = 0U;
    s_speed_drop_prev_abs_ref_rpm = 0U;
}

static void FOC_CheckSpeedDropFault(void)
{
    int16_t ref_rpm;
    int16_t fdb_rpm;
    int16_t ctrl_fdb_rpm;
    int16_t err_rpm;
    int16_t iq_ref_mA;
    uint16_t abs_ref;
    uint16_t abs_fdb;
    uint16_t abs_iq_ref;
    uint8_t ref_decreasing;
    uint8_t low_torque_no_edge;
    uint8_t bidir_no_edge_stop;
    uint8_t bidir_no_edge_guard;
    float signed_speed_fdb = s_ctx.speed_fdb;
    float signed_speed_ctrl_fdb = s_ctx.speed_ctrl_fdb;

    if ((g_foc_speed_drop_fault_enable == 0U) ||
        (s_ctx.fault != FOC_FAULT_NONE) ||
        ((g_foc_dyn_speed_enable == 0U) &&
         (g_foc_bidir_speed_enable == 0U))) {
        FOC_ResetSpeedDropFaultMonitor();
        return;
    }

    ref_rpm = g_foc_dyn_speed_ref_rpm;
    if ((ref_rpm < 0) && (s_ctx.direction == FOC_DIR_CCW)) {
        signed_speed_fdb = -signed_speed_fdb;
        signed_speed_ctrl_fdb = -signed_speed_ctrl_fdb;
    }

    fdb_rpm = FOC_Log_ToI16(signed_speed_fdb, 1.0f);
    ctrl_fdb_rpm = FOC_Log_ToI16(signed_speed_ctrl_fdb, 1.0f);
    err_rpm = FOC_Log_ToI16((float)ref_rpm - signed_speed_fdb, 1.0f);
    iq_ref_mA = FOC_Log_ToI16(FOC_ControlIqRef(), 1000.0f);
    abs_ref = (ref_rpm < 0) ? (uint16_t)(-ref_rpm) : (uint16_t)ref_rpm;
    abs_fdb = FOC_Log_ToU16(FOC_FABS(signed_speed_fdb), 1.0f);
    abs_iq_ref = (iq_ref_mA < 0)
               ? (uint16_t)(-iq_ref_mA)
               : (uint16_t)iq_ref_mA;
    ref_decreasing = (abs_ref < s_speed_drop_prev_abs_ref_rpm) ? 1U : 0U;
    low_torque_no_edge =
        ((s_ctx.sector_no_change_count >= FOC_SECTOR_NO_CHANGE_THRESHOLD) &&
         (abs_iq_ref < g_foc_speed_drop_fault_min_iq_ref_mA))
        ? 1U
        : 0U;
    bidir_no_edge_stop =
        ((g_foc_bidir_speed_enable != 0U) &&
         (s_ctx.sector_no_change_count >= FOC_SECTOR_NO_CHANGE_THRESHOLD))
        ? 1U
        : 0U;
    if (bidir_no_edge_stop != 0U) {
        s_speed_drop_bidir_no_edge_holdoff =
            g_foc_speed_drop_fault_bidir_no_edge_holdoff_cycles;
    } else if (s_speed_drop_bidir_no_edge_holdoff > 0U) {
        s_speed_drop_bidir_no_edge_holdoff--;
    }
    bidir_no_edge_guard =
        ((bidir_no_edge_stop != 0U) ||
         (s_speed_drop_bidir_no_edge_holdoff > 0U))
        ? 1U
        : 0U;
    g_foc_speed_drop_fault_bidir_no_edge_skip = bidir_no_edge_guard;
    g_foc_speed_drop_fault_bidir_no_edge_holdoff_count =
        s_speed_drop_bidir_no_edge_holdoff;
    if ((bidir_no_edge_guard != 0U) &&
        (g_foc_speed_drop_fault_bidir_no_edge_skip_count < 0xFFFFFFFFU)) {
        g_foc_speed_drop_fault_bidir_no_edge_skip_count++;
    }

    if ((ref_decreasing != 0U) &&
        (low_torque_no_edge == 0U) &&
        (bidir_no_edge_guard == 0U) &&
        (abs_ref >= g_foc_speed_drop_fault_ref_min_rpm) &&
        (abs_ref <= g_foc_speed_drop_fault_ref_max_rpm) &&
        (abs_ref > abs_fdb) &&
        ((uint16_t)(abs_ref - abs_fdb) >=
         g_foc_speed_drop_fault_err_rpm)) {
        if (g_foc_speed_drop_fault_count < 65535U) {
            g_foc_speed_drop_fault_count++;
        }
    } else {
        g_foc_speed_drop_fault_count = 0U;
    }

    g_foc_speed_drop_fault_ref_rpm = ref_rpm;
    g_foc_speed_drop_fault_fdb_rpm = fdb_rpm;
    g_foc_speed_drop_fault_ctrl_fdb_rpm = ctrl_fdb_rpm;
    g_foc_speed_drop_fault_err_last_rpm = err_rpm;
    g_foc_speed_drop_fault_iq_ref_mA = iq_ref_mA;
    g_foc_speed_drop_fault_sector_no_change_count =
        FOC_Log_U32ToU16((uint32_t)s_ctx.sector_no_change_count);
    g_foc_speed_drop_fault_edge_elapsed_us =
        FOC_HAL_GetTimestampUs() - s_ctx.timestamp_prev;
    g_foc_speed_drop_fault_direction = (uint8_t)s_ctx.direction;

    /* Keep the diagnostic counter visible, but do not latch a speed-drop fault. */

    s_speed_drop_prev_abs_ref_rpm = abs_ref;
}

static void FOC_ResetSpeedFdbDropFaultMonitor(void)
{
    g_foc_speed_fdb_drop_fault_count = 0U;
    g_foc_speed_fdb_drop_fault_ref_rpm = 0;
    g_foc_speed_fdb_drop_fault_fdb_rpm = 0;
    g_foc_speed_fdb_drop_fault_peak_rpm = 0U;
    g_foc_speed_fdb_drop_fault_drop_rpm = 0U;
    g_foc_speed_fdb_drop_fault_err_rpm = 0U;
    g_foc_speed_fdb_drop_fault_sector_no_change_count = 0U;
    g_foc_speed_fdb_drop_fault_edge_elapsed_us = 0U;
    g_foc_speed_fdb_drop_fault_direction = 0U;
    s_speed_fdb_drop_peak_rpm = 0U;
}

static void FOC_CheckSpeedFdbDropFault(void)
{
    uint16_t abs_ref;
    uint16_t abs_fdb;
    uint16_t drop_rpm;
    uint16_t err_rpm;
    uint32_t cur_ratio_scaled;
    uint32_t peak_ratio_scaled;
    float signed_ref = s_speed_ref_ctrl;

    if ((g_foc_speed_fdb_drop_fault_enable == 0U) ||
        (s_ctx.fault != FOC_FAULT_NONE) ||
        (s_ctx.state != FOC_STATE_RUNNING) ||
        (s_foc_ctrl_source != FOC_CTRL_SOURCE_SPEED)) {
        FOC_ResetSpeedFdbDropFaultMonitor();
        return;
    }

    if (s_ctx.direction == FOC_DIR_CCW) {
        signed_ref = -signed_ref;
    }

    abs_ref = FOC_Log_ToU16(FOC_FABS(s_speed_ref_ctrl), 1.0f);
    abs_fdb = FOC_Log_ToU16(FOC_FABS(s_ctx.speed_fdb), 1.0f);

    g_foc_speed_fdb_drop_fault_ref_rpm =
        FOC_Log_ToI16(signed_ref, 1.0f);
    g_foc_speed_fdb_drop_fault_fdb_rpm =
        FOC_Log_ToI16(s_ctx.speed_fdb, 1.0f);

    if (abs_ref < g_foc_speed_fdb_drop_fault_min_ref_rpm) {
        g_foc_speed_fdb_drop_fault_count = 0U;
        g_foc_speed_fdb_drop_fault_drop_rpm = 0U;
        g_foc_speed_fdb_drop_fault_err_rpm = 0U;
        s_speed_fdb_drop_peak_rpm = abs_fdb;
        g_foc_speed_fdb_drop_fault_peak_rpm = s_speed_fdb_drop_peak_rpm;
        return;
    }

    if (abs_fdb > s_speed_fdb_drop_peak_rpm) {
        s_speed_fdb_drop_peak_rpm = abs_fdb;
    }

    drop_rpm = (s_speed_fdb_drop_peak_rpm > abs_fdb)
             ? (uint16_t)(s_speed_fdb_drop_peak_rpm - abs_fdb)
             : 0U;
    err_rpm = (abs_ref > abs_fdb) ? (uint16_t)(abs_ref - abs_fdb) : 0U;

    cur_ratio_scaled = (uint32_t)abs_fdb * 100U;
    peak_ratio_scaled =
        (uint32_t)s_speed_fdb_drop_peak_rpm *
        (uint32_t)g_foc_speed_fdb_drop_fault_ratio_percent;

    if ((s_speed_fdb_drop_peak_rpm >=
         g_foc_speed_fdb_drop_fault_min_peak_rpm) &&
        (drop_rpm >= g_foc_speed_fdb_drop_fault_delta_rpm) &&
        (err_rpm >= g_foc_speed_fdb_drop_fault_delta_rpm) &&
        (cur_ratio_scaled <= peak_ratio_scaled)) {
        if (g_foc_speed_fdb_drop_fault_count < 65535U) {
            g_foc_speed_fdb_drop_fault_count++;
        }
    } else {
        g_foc_speed_fdb_drop_fault_count = 0U;
    }

    g_foc_speed_fdb_drop_fault_peak_rpm = s_speed_fdb_drop_peak_rpm;
    g_foc_speed_fdb_drop_fault_drop_rpm = drop_rpm;
    g_foc_speed_fdb_drop_fault_err_rpm = err_rpm;
    g_foc_speed_fdb_drop_fault_sector_no_change_count =
        FOC_Log_U32ToU16((uint32_t)s_ctx.sector_no_change_count);
    g_foc_speed_fdb_drop_fault_edge_elapsed_us =
        FOC_HAL_GetTimestampUs() - s_ctx.timestamp_prev;
    g_foc_speed_fdb_drop_fault_direction = (uint8_t)s_ctx.direction;

    /* Keep the diagnostic counter visible, but do not latch a speed-feedback-drop fault. */
}

static void FOC_UpdateSpeedControlFeedback(void)
{
    float alpha = FOC_SPEED_CTRL_FILTER_ALPHA;
    float ref_abs = FOC_FABS(s_speed_ref_ctrl);
    float smooth_max = (float)g_foc_low_speed_smooth_max_rpm;
    float overspeed_deadband = 0.0f;
    float tail_drop = (float)g_foc_bidir_zero_tail_fdb_drop_rpm;
    float tail_lead = (float)g_foc_bidir_zero_tail_fdb_lead_rpm;

    g_foc_low_speed_smooth_active = 0U;
    if ((g_foc_low_speed_smooth_enable != 0U) &&
        (ref_abs >= 1.0f) &&
        (smooth_max >= 1.0f) &&
        (ref_abs <= smooth_max)) {
        alpha = (float)g_foc_low_speed_ctrl_alpha_milli * 0.001f;
        overspeed_deadband =
            (float)g_foc_low_speed_overspeed_deadband_rpm;
        g_foc_low_speed_smooth_active = 1U;
    }

    alpha = FOC_CLAMP(alpha, 0.0f, 1.0f);

    if ((FOC_FABS(s_speed_ref_ctrl) < 1.0f) &&
        (FOC_FABS(s_ctx.speed_fdb) < 1.0f)) {
        if (FOC_FABS(s_ctx.speed_ctrl_fdb) < 1.0f) {
            s_ctx.speed_ctrl_fdb = 0.0f;
        } else {
            FOC_DecaySpeedControlFeedback(0.0f);
        }
    } else if ((FOC_FABS(s_ctx.speed_ctrl_fdb) < 1.0f) &&
               (FOC_FABS(s_ctx.speed_fdb) >= 1.0f)) {
        s_ctx.speed_ctrl_fdb = s_ctx.speed_fdb;
    } else if (s_ctx.speed_fdb > (s_speed_ref_ctrl + overspeed_deadband)) {
        s_ctx.speed_ctrl_fdb = s_ctx.speed_fdb;
    } else if (s_ctx.speed_fdb < s_ctx.speed_ctrl_fdb) {
        if ((g_foc_bidir_zero_tail_active != 0U) &&
            (tail_drop > 0.0f) &&
            ((s_ctx.speed_ctrl_fdb - s_ctx.speed_fdb) >= tail_drop)) {
            if (tail_lead < 0.0f) {
                tail_lead = 0.0f;
            }
            s_ctx.speed_ctrl_fdb = s_ctx.speed_fdb + tail_lead;
            if (s_ctx.speed_ctrl_fdb < s_ctx.speed_fdb) {
                s_ctx.speed_ctrl_fdb = s_ctx.speed_fdb;
            }
            if (g_foc_bidir_zero_tail_fdb_catch_count < 0xFFFFFFFFU) {
                g_foc_bidir_zero_tail_fdb_catch_count++;
            }
        } else {
            FOC_DecaySpeedControlFeedback(s_ctx.speed_fdb);
        }
    } else if (alpha >= 1.0f) {
        s_ctx.speed_ctrl_fdb = s_ctx.speed_fdb;
    } else if (alpha > 0.0f) {
        s_ctx.speed_ctrl_fdb =
            alpha * s_ctx.speed_fdb + (1.0f - alpha) * s_ctx.speed_ctrl_fdb;
    }

    g_foc_speed_ctrl_fdb_rpm = FOC_Log_ToI16(s_ctx.speed_ctrl_fdb, 1.0f);
}

static void FOC_Prof_Reset(void)
{
     g_foc_prof_enter_us = 0U;
     g_foc_prof_after_state_us = 0U;
     g_foc_prof_after_hall_us = 0U;
     g_foc_prof_after_adc_us = 0U;
     g_foc_prof_after_calc_us = 0U;
     g_foc_prof_after_pwm_us = 0U;
     g_foc_prof_exit_us = 0U;
     g_foc_prof_period_us = 0U;
     g_foc_prof_loop_us = 0U;
     g_foc_prof_max_period_us = 0U;
     g_foc_prof_max_loop_us = 0U;
     g_foc_prof_loop_count = 0U;
     g_foc_prof_max_period_loop = 0U;
     g_foc_prof_max_loop_loop = 0U;
     g_foc_prof_max_seg_loop = 0U;
     g_foc_prof_last_seg_us = 0U;
     g_foc_prof_max_seg_us = 0U;
     g_foc_prof_last_seg_id = 0U;
     g_foc_prof_max_seg_id = 0U;
     g_foc_late_period_count = 0U;
     g_foc_late_period_us = 0U;
     g_foc_hall_resync_count = 0U;
     g_foc_hall_resync_period_us = 0U;
     g_foc_hall_resync_prev_sector = 0U;
     g_foc_hall_resync_cur_sector = 0U;
     g_foc_late_recovery_count = 0U;
     g_foc_late_recovery_period_us = 0U;
     g_foc_hall_recovery_accept_count = 0U;
     g_foc_hall_recovery_accept_prev_sector = 0U;
     g_foc_hall_recovery_accept_cur_sector = 0U;
     g_foc_post_recovery_duty_slew_count = 0U;
     g_foc_post_recovery_duty_slew_remaining = 0U;
     g_foc_post_recovery_duty_slew_active = 0U;
     g_foc_recovery_zero_vector_count = 0U;
     g_foc_recovery_pwm_off_count = 0U;
     g_foc_recovery_pwm_hold_count = 0U;
     g_foc_recovery_current_control_count = 0U;
     g_foc_recovery_zero_vector_remaining = 0U;
     g_foc_recovery_current_wait_count = 0U;
     g_foc_recovery_release_current_mA = 0U;
     g_foc_recovery_current_wait_active = 0U;
     g_foc_hall_min_time_reject_count = 0U;
     g_foc_hall_min_time_last_elapsed_us = 0U;
     g_foc_hall_min_time_last_min_us = 0U;
     g_foc_hall_min_time_prev_sector = 0U;
     g_foc_hall_min_time_cur_sector = 0U;
     g_foc_hall_event_used_count = 0U;
     g_foc_hall_event_seq = 0U;
     g_foc_hall_event_age_us = 0U;
     g_foc_hall_poll_count = 0U;
     g_foc_speed_ctrl_fdb_rpm = 0;
     g_foc_speed_error_boost_mA = 0;
     FOC_ResetSpeedDropFaultMonitor();
     FOC_ResetSpeedFdbDropFaultMonitor();
     g_foc_signed_speed_ref_normalize_count = 0U;
     g_foc_vbus_brake_limit_mA = 0;
     g_foc_vbus_brake_active = 0U;
     g_foc_vbus_brake_limited_count = 0U;
     s_foc_prof_last_enter_us = 0U;
     s_foc_control_period_us = FOC_CONTROL_PERIOD_US;
     s_hall_recovery_accept_cycles = 0U;
     s_post_recovery_duty_slew_cycles = 0U;
     s_recovery_zero_vector_cycles = 0U;
     s_recovery_zero_vector_min_cycles = 0U;
     s_speed_loop_accum_us = 0U;
     s_speed_error_boost_prev_ref = 0.0f;
     s_hall_event_seq_seen = 0U;
 }

 static void FOC_LastFault_Reset(void)
 {
     g_foc_last_fault_latched = 0U;
     g_foc_last_fault_state = 0U;
     g_foc_last_fault_latch_count = 0U;
     g_foc_last_fault_seq = 0U;
     g_foc_last_fault_current_peak_mA = 0U;
     g_foc_last_fault_vbus_mV = 0U;
     g_foc_last_fault_speed_ref_rpm = 0U;
     g_foc_last_fault_speed_fdb_rpm = 0U;
     g_foc_last_fault_speed_ctrl_fdb_rpm = 0;
     g_foc_last_fault_iq_ref_mA = 0;
     g_foc_last_fault_iq_mA = 0;
     g_foc_last_fault_sector_no_change_count = 0U;
     g_foc_last_fault_stall_counter = 0U;
     g_foc_last_fault_direction = 0U;
     g_foc_last_fault_hall_raw = 0U;
     g_foc_last_fault_hall_sector = 0U;
     g_foc_last_fault_theta_hall = 0U;
     g_foc_last_fault_theta_pred = 0U;
     g_foc_last_fault_theta_ctrl = 0U;
 }

 static uint32_t FOC_Prof_Enter(void)
 {
     uint32_t now = FOC_HAL_GetTimestampUs();
     uint32_t period = 0U;

     g_foc_prof_loop_count++;

     if (s_foc_prof_last_enter_us != 0U) {
         period = now - s_foc_prof_last_enter_us;
         if (period > g_foc_prof_max_period_us) {
             g_foc_prof_max_period_us = period;
             g_foc_prof_max_period_loop = g_foc_prof_loop_count;
         }
     }

     s_foc_prof_last_enter_us = now;
     g_foc_prof_enter_us = now;
     g_foc_prof_period_us = period;

     return now;
 }

 static void FOC_Prof_RecordSegment(uint32_t start_us, uint32_t end_us, uint8_t seg_id)
 {
     uint32_t seg_us = end_us - start_us;

     g_foc_prof_last_seg_us = seg_us;
     g_foc_prof_last_seg_id = seg_id;

     if (seg_us > g_foc_prof_max_seg_us) {
         g_foc_prof_max_seg_us = seg_us;
         g_foc_prof_max_seg_id = seg_id;
         g_foc_prof_max_seg_loop = g_foc_prof_loop_count;
     }
 }

 static void FOC_Prof_Exit(uint32_t enter_us, uint32_t exit_us)
 {
     uint32_t loop_us = exit_us - enter_us;

     g_foc_prof_exit_us = exit_us;
     g_foc_prof_loop_us = loop_us;

     if (loop_us > g_foc_prof_max_loop_us) {
         g_foc_prof_max_loop_us = loop_us;
         g_foc_prof_max_loop_loop = g_foc_prof_loop_count;
     }
 }

 static uint32_t FOC_ControlPeriodUs(void)
 {
     uint32_t period_us = g_foc_prof_period_us;

     if (period_us == 0U) {
         period_us = FOC_CONTROL_PERIOD_US;
     }

     s_foc_control_period_us = period_us;
     return period_us;
 }

 static uint8_t FOC_ControlPeriodIsLate(uint32_t period_us)
 {
     if (period_us > FOC_CONTROL_LATE_PERIOD_US) {
         g_foc_late_period_count++;
         g_foc_late_period_us = period_us;
         return 1U;
     }

     return 0U;
 }

 static float FOC_ControlDtFromUs(uint32_t period_us, uint32_t max_us)
 {
     if (period_us == 0U) {
         period_us = FOC_CONTROL_PERIOD_US;
     }
     if (period_us > max_us) {
         period_us = max_us;
     }

     return (float)period_us * 1.0e-6f;
 }

 static float FOC_ControlInvDtFromUs(uint32_t period_us, uint32_t max_us)
 {
     if (period_us == 0U) {
         period_us = FOC_CONTROL_PERIOD_US;
     }
     if (period_us > max_us) {
         period_us = max_us;
     }

     return 1000000.0f / (float)period_us;
 }

 static uint8_t FOC_ControlPeriodNeedsRecovery(uint32_t period_us)
 {
     if (period_us > FOC_CONTROL_RECOVERY_PERIOD_US) {
         g_foc_late_recovery_count++;
         g_foc_late_recovery_period_us = period_us;
#if FOC_CONTROL_RECOVERY_ENABLE
         return 1U;
#endif
     }

     return 0U;
 }

 static void FOC_ResetClosedLoopForRecovery(void)
 {
     FOC_PID_Reset(&s_ctx.pid_speed);
     FOC_PID_Reset(&s_ctx.pid_id);
     FOC_PID_Reset(&s_ctx.pid_iq);
     FOC_ResetCurrentAngleTrim();

     s_ctx.iq_ref = 0.0f;
     s_ctx.speed_ctrl_fdb = 0.0f;
     g_foc_speed_ctrl_fdb_rpm = 0;
     s_ctx.speed_loop_counter = 0U;
     s_speed_loop_accum_us = 0U;
     s_speed_error_boost_prev_ref = 0.0f;
     FOC_ResetSpeedRefRamp();

     s_ctx.v_dq.d = 0.0f;
     s_ctx.v_dq.q = 0.0f;
     s_ctx.v_ab.alpha = 0.0f;
     s_ctx.v_ab.beta = 0.0f;
 }

 static void FOC_BeginRecoveryZeroVectorHold(void)
 {
     uint16_t min_cycles = FOC_RECOVERY_ZERO_VECTOR_MIN_CYCLES;
     uint16_t max_cycles = FOC_RECOVERY_ZERO_VECTOR_MAX_CYCLES;

     if (max_cycles < min_cycles) {
         max_cycles = min_cycles;
     }

     s_recovery_zero_vector_cycles = max_cycles;
     s_recovery_zero_vector_min_cycles = min_cycles;
     g_foc_recovery_zero_vector_remaining = s_recovery_zero_vector_cycles;
     g_foc_recovery_current_wait_active = 1U;
 }

 static void FOC_ServiceRecoveryZeroVectorHold(void)
 {
     uint8_t release_after_this_cycle = 0U;

     if (s_recovery_zero_vector_cycles == 0U) {
         g_foc_recovery_zero_vector_remaining = 0U;
         g_foc_recovery_current_wait_active = 0U;
         return;
     }

     if (s_recovery_zero_vector_min_cycles > 0U) {
         s_recovery_zero_vector_min_cycles--;
     } else if (s_ctx.current_peak <= FOC_RECOVERY_RELEASE_CURRENT_A) {
         release_after_this_cycle = 1U;
     } else {
         g_foc_recovery_current_wait_count++;
     }

     g_foc_recovery_zero_vector_count++;

     if (s_recovery_zero_vector_cycles > 0U) {
         s_recovery_zero_vector_cycles--;
     }

     if ((release_after_this_cycle != 0U) ||
         (s_recovery_zero_vector_cycles == 0U)) {
         s_recovery_zero_vector_cycles = 0U;
         s_recovery_zero_vector_min_cycles = 0U;
         g_foc_recovery_release_current_mA =
             FOC_Log_ToU16(s_ctx.current_peak, 1000.0f);
     }

     g_foc_recovery_zero_vector_remaining = s_recovery_zero_vector_cycles;
     g_foc_recovery_current_wait_active =
         (s_recovery_zero_vector_cycles > 0U) ? 1U : 0U;
 }

 static void FOC_BeginPostRecoveryDutySlew(void)
 {
     s_post_recovery_duty_slew_cycles = FOC_POST_RECOVERY_DUTY_SLEW_CYCLES;
     g_foc_post_recovery_duty_slew_remaining = s_post_recovery_duty_slew_cycles;
     g_foc_post_recovery_duty_slew_active =
         (s_post_recovery_duty_slew_cycles > 0U) ? 1U : 0U;
 }

 static void FOC_RunRecoveryCurrentControl(float theta_ctrl,
                                           float pid_dt,
                                           float pid_inv_dt)
 {
     float duty_prev_a = s_ctx.duty_a;
     float duty_prev_b = s_ctx.duty_b;
     float duty_prev_c = s_ctx.duty_c;

     s_ctx.iq_ref = 0.0f;
     s_ctx.speed_loop_counter = 0U;
     s_speed_loop_accum_us = 0U;
     s_speed_error_boost_prev_ref = 0.0f;


     s_ctx.v_dq.d = FOC_PID_Update(&s_ctx.pid_id,
                                   s_ctx.id_ref - s_ctx.i_dq.d,
                                   pid_dt,
                                   pid_inv_dt);
     s_ctx.v_dq.q = FOC_PID_Update(&s_ctx.pid_iq,
                                   -s_ctx.i_dq.q,
                                   pid_dt,
                                   pid_inv_dt);

     FOC_InvPark(&s_ctx.v_dq, theta_ctrl, &s_ctx.v_ab);
     FOC_SVPWM_Calculate(s_ctx.v_ab.alpha, s_ctx.v_ab.beta, s_ctx.v_bus,
                         &s_ctx.duty_a, &s_ctx.duty_b, &s_ctx.duty_c);
     FOC_ApplyPostRecoveryDutySlew(duty_prev_a, duty_prev_b, duty_prev_c);

     g_foc_recovery_current_control_count++;
     FOC_HAL_SetDutyCycle(s_ctx.duty_a, s_ctx.duty_b, s_ctx.duty_c);
 }

 static float FOC_LimitDutyStep(float target, float previous)
 {
     float max_step = FOC_POST_RECOVERY_DUTY_STEP_MAX;

     if (max_step > 0.0f) {
         float upper = previous + max_step;
         float lower = previous - max_step;

         if (target > upper) {
             target = upper;
         } else if (target < lower) {
             target = lower;
         }
     }

     return FOC_CLAMP(target, 0.0f, 1.0f);
 }

 static void FOC_ApplyPostRecoveryDutySlew(float prev_a,
                                           float prev_b,
                                           float prev_c)
 {
     if (s_post_recovery_duty_slew_cycles == 0U) {
         g_foc_post_recovery_duty_slew_active = 0U;
         g_foc_post_recovery_duty_slew_remaining = 0U;
         return;
     }

     s_ctx.duty_a = FOC_LimitDutyStep(s_ctx.duty_a, prev_a);
     s_ctx.duty_b = FOC_LimitDutyStep(s_ctx.duty_b, prev_b);
     s_ctx.duty_c = FOC_LimitDutyStep(s_ctx.duty_c, prev_c);

     s_post_recovery_duty_slew_cycles--;
     g_foc_post_recovery_duty_slew_count++;
     g_foc_post_recovery_duty_slew_remaining = s_post_recovery_duty_slew_cycles;
     g_foc_post_recovery_duty_slew_active =
         (s_post_recovery_duty_slew_cycles > 0U) ? 1U : 0U;
 }

 static void FOC_Log_Reset(void)
 {
     g_foc_log_idx = 0U;
     g_foc_log_fault_idx = 0U;
     g_foc_log_stop = 0U;
     g_foc_log_seq = 0U;
     g_log_idx = 0U;
     g_log_fault_idx = 0U;
     s_foc_log_decim = 0U;
     s_hall_illegal_transition_count = 0U;
     FOC_Prof_Reset();
 }

 static void FOC_Log_Record(float theta_ctrl)
 {
     uint16_t idx;
     uint16_t text_idx;
     volatile FOC_LogSample_t *p;

     if (g_foc_log_stop != 0U) {
         return;
     }

     s_foc_log_decim++;
     if (s_foc_log_decim < FOC_LOG_DECIMATION) {
         return;
     }
     s_foc_log_decim = 0U;

     idx = g_foc_log_idx;
     p = &g_foc_log[idx];

     p->seq = g_foc_log_seq++;
     p->t_us = FOC_HAL_GetTimestampUs();

     p->speed_ref_rpm = FOC_Log_ToI16(FOC_SignedSpeedRef(), 1.0f);
     p->speed_fdb_rpm = FOC_Log_ToI16(s_ctx.speed_fdb, 1.0f);
     p->speed_ctrl_fdb_rpm = FOC_Log_ToI16(s_ctx.speed_ctrl_fdb, 1.0f);

     p->ia_mA = FOC_Log_ToI16(s_ctx.i_abc.ia, 1000.0f);
     p->ib_mA = FOC_Log_ToI16(s_ctx.i_abc.ib, 1000.0f);
     p->ic_mA = FOC_Log_ToI16(s_ctx.i_abc.ic, 1000.0f);
     p->id_mA = FOC_Log_ToI16(s_ctx.i_dq.d, 1000.0f);
     p->iq_mA = FOC_Log_ToI16(s_ctx.i_dq.q, 1000.0f);
     p->id_ref_mA = FOC_Log_ToI16(s_ctx.id_ref, 1000.0f);
     p->iq_ref_mA = FOC_Log_ToI16(FOC_ControlIqRef(), 1000.0f);
     p->speed_error_boost_mA = g_foc_speed_error_boost_mA;

     p->vd_mV = FOC_Log_ToI16(s_ctx.v_dq.d, 1000.0f);
     p->vq_mV = FOC_Log_ToI16(s_ctx.v_dq.q, 1000.0f);
     p->valpha_mV = FOC_Log_ToI16(s_ctx.v_ab.alpha, 1000.0f);
     p->vbeta_mV = FOC_Log_ToI16(s_ctx.v_ab.beta, 1000.0f);

     p->vbus_mV = FOC_Log_ToU16(s_ctx.v_bus, 1000.0f);
     p->duty_a = FOC_Log_ToU16(s_ctx.duty_a, 10000.0f);
     p->duty_b = FOC_Log_ToU16(s_ctx.duty_b, 10000.0f);
     p->duty_c = FOC_Log_ToU16(s_ctx.duty_c, 10000.0f);

     p->theta_hall_u16 = FOC_Log_AngleU16(s_ctx.theta_e);
     p->theta_pred_u16 = FOC_Log_AngleU16(s_ctx.theta_e_predicted);
     p->theta_ctrl_u16 = FOC_Log_AngleU16(theta_ctrl);

     p->current_peak_mA = FOC_Log_ToU16(s_ctx.current_peak, 1000.0f);
     p->sector_no_change_count =
         FOC_Log_U32ToU16((uint32_t)s_ctx.sector_no_change_count);
     p->stall_counter = FOC_Log_U32ToU16(s_ctx.stall_counter);

     p->hall_raw = (uint8_t)((s_ctx.hall_raw.h1 << 2)
                           | (s_ctx.hall_raw.h2 << 1)
                           |  s_ctx.hall_raw.h3);
     p->hall_sector = s_ctx.hall_sector.sector;
     p->direction = (uint8_t)s_ctx.direction;
     p->state = (uint8_t)s_ctx.state;
     p->fault = (uint8_t)s_ctx.fault;

     g_foc_log_fault_idx = idx;

     text_idx = g_log_idx;
     g_log_seq[text_idx] = p->seq;
     g_log_t_us[text_idx] = p->t_us;
     g_log_speed_ref_rpm[text_idx] = p->speed_ref_rpm;
     g_log_speed_fdb_rpm[text_idx] = p->speed_fdb_rpm;
     g_log_speed_ctrl_fdb_rpm[text_idx] = p->speed_ctrl_fdb_rpm;
     g_log_ia_mA[text_idx] = p->ia_mA;
     g_log_ib_mA[text_idx] = p->ib_mA;
     g_log_ic_mA[text_idx] = p->ic_mA;
     g_log_id_mA[text_idx] = p->id_mA;
     g_log_iq_mA[text_idx] = p->iq_mA;
     g_log_id_ref_mA[text_idx] = p->id_ref_mA;
     g_log_iq_ref_mA[text_idx] = p->iq_ref_mA;
     g_log_speed_error_boost_mA[text_idx] = p->speed_error_boost_mA;
     g_log_vd_mV[text_idx] = p->vd_mV;
     g_log_vq_mV[text_idx] = p->vq_mV;
     g_log_vbus_mV[text_idx] = p->vbus_mV;
     g_log_duty_a[text_idx] = p->duty_a;
     g_log_duty_b[text_idx] = p->duty_b;
     g_log_duty_c[text_idx] = p->duty_c;
     g_log_theta_hall[text_idx] = p->theta_hall_u16;
     g_log_theta_pred[text_idx] = p->theta_pred_u16;
     g_log_theta_ctrl[text_idx] = p->theta_ctrl_u16;
     g_log_current_peak_mA[text_idx] = p->current_peak_mA;
     g_log_sector_no_change_count[text_idx] = p->sector_no_change_count;
     g_log_stall_counter[text_idx] = p->stall_counter;
     g_log_hall_raw[text_idx] = p->hall_raw;
     g_log_hall_sector[text_idx] = p->hall_sector;
     g_log_direction[text_idx] = p->direction;
     g_log_state[text_idx] = p->state;
     g_log_fault[text_idx] = p->fault;

     g_log_fault_idx = text_idx;

     text_idx++;
     if (text_idx >= FOC_TEXT_LOG_SIZE) {
         text_idx = 0U;
     }
     g_log_idx = text_idx;

     idx++;
     if (idx >= FOC_LOG_SIZE) {
         idx = 0U;
     }
     g_foc_log_idx = idx;

     if (s_ctx.fault != FOC_FAULT_NONE) {
         g_foc_last_fault_theta_ctrl = p->theta_ctrl_u16;
         g_foc_log_stop = 1U;
     }
 }

 static void FOC_EnterFaultState(void)
 {
     if (s_ctx.fault != FOC_FAULT_NONE) {
         g_foc_last_fault_latched = (uint8_t)s_ctx.fault;
         g_foc_last_fault_state = (uint8_t)s_ctx.state;
         g_foc_last_fault_latch_count++;
         g_foc_last_fault_seq = g_foc_log_seq;
         g_foc_last_fault_current_peak_mA =
             (uint32_t)FOC_Log_ToU16(s_ctx.current_peak, 1000.0f);
         g_foc_last_fault_vbus_mV = g_foc_vbus_mV;
         g_foc_last_fault_speed_ref_rpm =
             FOC_Log_ToU16(s_ctx.speed_ref, 1.0f);
         g_foc_last_fault_speed_fdb_rpm =
             FOC_Log_ToU16(s_ctx.speed_fdb, 1.0f);
         g_foc_last_fault_speed_ctrl_fdb_rpm =
             FOC_Log_ToI16(s_ctx.speed_ctrl_fdb, 1.0f);
         g_foc_last_fault_iq_ref_mA =
             FOC_Log_ToI16(FOC_ControlIqRef(), 1000.0f);
         g_foc_last_fault_iq_mA =
             FOC_Log_ToI16(s_ctx.i_dq.q, 1000.0f);
         g_foc_last_fault_sector_no_change_count =
             FOC_Log_U32ToU16((uint32_t)s_ctx.sector_no_change_count);
         g_foc_last_fault_stall_counter =
             FOC_Log_U32ToU16(s_ctx.stall_counter);
         g_foc_last_fault_direction = (uint8_t)s_ctx.direction;
         g_foc_last_fault_hall_raw =
             (uint8_t)((s_ctx.hall_raw.h1 << 2)
                     | (s_ctx.hall_raw.h2 << 1)
                     |  s_ctx.hall_raw.h3);
         g_foc_last_fault_hall_sector = s_ctx.hall_sector.sector;
         g_foc_last_fault_theta_hall = FOC_Log_AngleU16(s_ctx.theta_e);
         g_foc_last_fault_theta_pred =
             FOC_Log_AngleU16(s_ctx.theta_e_predicted);
         g_foc_last_fault_theta_ctrl = g_foc_last_fault_theta_pred;
     }

     FOC_HAL_DisablePWM();
     FOC_HAL_SetDutyCycle(0.0f, 0.0f, 0.0f);

     s_ctx.duty_a = 0.0f;
     s_ctx.duty_b = 0.0f;
     s_ctx.duty_c = 0.0f;
     s_recovery_zero_vector_cycles = 0U;
     s_recovery_zero_vector_min_cycles = 0U;
     g_foc_recovery_zero_vector_remaining = 0U;
     g_foc_recovery_current_wait_active = 0U;
     s_ctx.state = FOC_STATE_FAULT;
 }

 static uint8_t FOC_HallSectorsAreAdjacent(uint8_t from, uint8_t to)
 {
     uint8_t next;
     uint8_t prev;

     if ((from < 1U) || (from > 6U) || (to < 1U) || (to > 6U)) {
         return 0U;
     }

     next = (from == 6U) ? 1U : (uint8_t)(from + 1U);
     prev = (from == 1U) ? 6U : (uint8_t)(from - 1U);

     return (uint8_t)((to == next) || (to == prev));
 }

 static int16_t FOC_ClampI32ToI16(int32_t v)
 {
     if (v > 32767) {
         return 32767;
     }
     if (v < -32768) {
         return -32768;
     }
     return (int16_t)v;
 }

 static uint8_t FOC_Core_HallRawToU8(const FOC_HallRaw_t *hall_raw)
 {
     return (uint8_t)((hall_raw->h1 << 2) |
                      (hall_raw->h2 << 1) |
                      hall_raw->h3);
 }

 static uint32_t FOC_HallEdgeDeltaUs(uint32_t sector_timestamp_us)
 {
     return (s_ctx.timestamp_prev != 0U)
          ? (sector_timestamp_us - s_ctx.timestamp_prev)
          : 0U;
 }

 static void FOC_RecordHallHistory(uint32_t delta_time_us, int32_t hall_delta)
 {
     uint8_t motor = s_foc_core_active_motor;
     uint16_t next;

     if (motor >= FOC_CORE_MOTOR_COUNT) {
         return;
     }

     next = (s_foc_hall_history_head[motor] < 0)
          ? 0U
          : (uint16_t)(s_foc_hall_history_head[motor] + 1);
     if (next >= FOC_HALL_HISTORY_SIZE) {
         next = 0U;
     }

     if (s_foc_hall_total_update_count[motor] < 0xFFFFFFFFFFFFFFFFULL) {
         s_foc_hall_total_update_count[motor]++;
     }

     if (hall_delta != 0) {
         if (((s_foc_hall_last_delta[motor] > 0) && (hall_delta < 0)) ||
             ((s_foc_hall_last_delta[motor] < 0) && (hall_delta > 0))) {
             s_foc_hall_direction_update_count[motor] = 0U;
         }
         if (s_foc_hall_direction_update_count[motor] < 0xFFFFFFFFFFFFFFFFULL) {
             s_foc_hall_direction_update_count[motor]++;
         }
         s_foc_hall_last_delta[motor] = hall_delta;
     }

     s_foc_hall_history_delta_us[motor][next] = delta_time_us;
     s_foc_hall_history_raw[motor][next] = FOC_Core_HallRawToU8(&s_ctx.hall_raw);
     s_foc_hall_history_count[motor][next] = s_foc_hall_total_update_count[motor];
     s_foc_hall_history_head[motor] = (int16_t)next;

     if (s_foc_hall_history_valid_count[motor] < FOC_HALL_HISTORY_SIZE) {
         s_foc_hall_history_valid_count[motor]++;
     }
 }

 static int32_t FOC_RecordHallTravelStep(uint8_t prev_sector, uint8_t cur_sector)
 {
     uint8_t inc_steps;
     uint8_t dec_steps;
     int32_t delta;

     if ((prev_sector < 1U) || (prev_sector > 6U) ||
         (cur_sector < 1U) || (cur_sector > 6U) ||
         (prev_sector == cur_sector)) {
         return 0;
     }

     inc_steps = (uint8_t)((cur_sector + 6U - prev_sector) % 6U);
     dec_steps = (uint8_t)((prev_sector + 6U - cur_sector) % 6U);

     if (inc_steps == 0U) {
         inc_steps = 6U;
     }
     if (dec_steps == 0U) {
         dec_steps = 6U;
     }

     /* External travel convention: CCW is positive, CW is negative. */
#if FOC_FORWARD_HALL_DIR
     delta = (inc_steps <= dec_steps) ? (int32_t)inc_steps : -(int32_t)dec_steps;
#else
     delta = (inc_steps <= dec_steps) ? -(int32_t)inc_steps : (int32_t)dec_steps;
#endif
     delta = FOC_FilterHallTravelDelta(delta);
     g_foc_hall_travel_count[s_foc_core_active_motor] += delta;
     return delta;
 }

 static uint32_t FOC_HallMinSectorTimeUs(void)
 {
     float max_rpm = FOC_HALL_MIN_SECTOR_TIME_MAX_RPM;
     float pole_pairs = (s_config.motor.pole_pairs > 0U)
                       ? (float)s_config.motor.pole_pairs
                       : 1.0f;
     float min_us;

     if (max_rpm < 1.0f) {
         return 1U;
     }

     min_us = 10000000.0f / (max_rpm * pole_pairs);
     min_us *= FOC_HALL_MIN_SECTOR_TIME_RATIO;

     if (min_us < 1.0f) {
         return 1U;
     }
     if (min_us > 4294967295.0f) {
         return 0xFFFFFFFFU;
     }

     return (uint32_t)min_us;
 }

 static uint8_t FOC_ApplyHallSector(const FOC_HallSector_t *candidate,
                                    uint32_t timestamp_us,
                                    uint8_t timestamp_valid,
                                    uint8_t allow_missed_transition)
{
     uint8_t cur_sector = candidate->sector;
     uint8_t prev_sector = s_ctx.hall_sector_prev;
     uint8_t recovery_accept = (s_hall_recovery_accept_cycles > 0U) ? 1U : 0U;
     uint32_t sector_timestamp_us = (timestamp_valid != 0U)
                                  ? timestamp_us
                                  : FOC_HAL_GetTimestampUs();

     if (cur_sector == 0U) {
         return 0U;
     }

     if (s_hall_recovery_accept_cycles > 0U) {
         s_hall_recovery_accept_cycles--;
     }

     if ((prev_sector == 0U) || (cur_sector == prev_sector)) {
         s_ctx.hall_sector = *candidate;
         if (prev_sector == 0U) {
             s_ctx.hall_sector_timestamp_us = sector_timestamp_us;
             FOC_RecordHallHistory(0U, 0);
         }
         s_hall_illegal_transition_count = 0U;
         return 1U;
     }

     if (allow_missed_transition != 0U) {
         int32_t hall_delta = FOC_RecordHallTravelStep(prev_sector, cur_sector);
         s_ctx.hall_sector = *candidate;
         s_ctx.hall_sector_timestamp_us = sector_timestamp_us;
         FOC_RecordHallHistory(FOC_HallEdgeDeltaUs(sector_timestamp_us), hall_delta);
         s_hall_illegal_transition_count = 0U;
         g_foc_hall_resync_count++;
         g_foc_hall_resync_period_us = (s_foc_control_period_us > 65535U)
                                     ? 65535U
                                     : (uint16_t)s_foc_control_period_us;
         g_foc_hall_resync_prev_sector = prev_sector;
         g_foc_hall_resync_cur_sector = cur_sector;
         return 1U;
     }

     if (FOC_HallSectorsAreAdjacent(prev_sector, cur_sector) == 0U) {
         if (s_hall_illegal_transition_count < 65535U) {
             s_hall_illegal_transition_count++;
         }
         if (s_hall_illegal_transition_count >= FOC_HALL_ILLEGAL_TRANSITION_FAULT_COUNT) {
             s_ctx.fault |= FOC_FAULT_HALL;
         }
         return 0U;
     }

     if (recovery_accept != 0U) {
         int32_t hall_delta = FOC_RecordHallTravelStep(prev_sector, cur_sector);
         s_ctx.hall_sector = *candidate;
         s_ctx.hall_sector_timestamp_us = sector_timestamp_us;
         FOC_RecordHallHistory(FOC_HallEdgeDeltaUs(sector_timestamp_us), hall_delta);
         s_ctx.theta_e_predicted = candidate->theta_e;
         s_hall_illegal_transition_count = 0U;
         g_foc_hall_recovery_accept_count++;
         g_foc_hall_recovery_accept_prev_sector = prev_sector;
         g_foc_hall_recovery_accept_cur_sector = cur_sector;
         return 1U;
     }

     {
         uint32_t elapsed_us = sector_timestamp_us - s_ctx.timestamp_prev;
         uint32_t min_us = FOC_HallMinSectorTimeUs();

         if (elapsed_us < min_us) {
             g_foc_hall_min_time_reject_count++;
             g_foc_hall_min_time_last_elapsed_us = elapsed_us;
             g_foc_hall_min_time_last_min_us = min_us;
             g_foc_hall_min_time_prev_sector = prev_sector;
             g_foc_hall_min_time_cur_sector = cur_sector;
             return 0U;
         }
     }

     {
         int32_t hall_delta = FOC_RecordHallTravelStep(prev_sector, cur_sector);
         s_ctx.hall_sector = *candidate;
         s_ctx.hall_sector_timestamp_us = sector_timestamp_us;
         FOC_RecordHallHistory(FOC_HallEdgeDeltaUs(sector_timestamp_us), hall_delta);
     }
     s_hall_illegal_transition_count = 0U;
     return 1U;
 }

 

 /* ===================================================================

  *  初始化 / 反初始化

  * =================================================================== */

 

 int FOC_Core_Init(const FOC_Config_t *config)

 {
     uint8_t motor;

     if (config == NULL) {

         return FOC_ERR;

     }

 

     /* 初始化驱动层 */

     if (FOC_HAL_Init() != 0) {

         return FOC_ERR;

     }

 

     /* 初始化 sin/cos 查找表 */

     FOC_Math_InitTable();

     for (motor = 0U; motor < FOC_CORE_MOTOR_COUNT; motor++) {
         g_foc_hall_travel_count[motor] = 0;
         g_foc_hall_travel_offset[motor] = 0;
         memset(s_foc_hall_history_delta_us[motor], 0,
                sizeof(s_foc_hall_history_delta_us[motor]));
         memset(s_foc_hall_history_raw[motor], 0,
                sizeof(s_foc_hall_history_raw[motor]));
         memset(s_foc_hall_history_count[motor], 0,
                sizeof(s_foc_hall_history_count[motor]));
         s_foc_hall_history_head[motor] = -1;
         s_foc_hall_history_valid_count[motor] = 0U;
         s_foc_hall_total_update_count[motor] = 0U;
         s_foc_hall_direction_update_count[motor] = 0U;
         s_foc_hall_last_delta[motor] = 0;
         s_foc_hall_travel_stall_active_store[motor] = 0U;
         s_foc_hall_travel_stall_counter_store[motor] = 0U;
         s_foc_hall_travel_stall_dir_store[motor] = 0;
         s_foc_hall_travel_suppressed_delta_store[motor] = 0;
         g_foc_hall_travel_stall_active[motor] = 0U;
         g_foc_hall_travel_stall_counter[motor] = 0U;
         g_foc_hall_travel_stall_dir[motor] = 0;
         g_foc_hall_travel_suppressed_delta[motor] = 0;
         g_foc_hall_travel_freeze_count[motor] = 0U;
     }

     FOC_Core_SelectMotor(0U);

 

     /* 保存配置 */

     memcpy(&s_config, config, sizeof(FOC_Config_t));

 

     /* 从驱动层获取标定参数并缓存，避免 ISR 热路径中反复调用 HAL */

     FOC_HAL_GetCurrentOffset(&s_config.current_calib);

     s_config.current_calib.i_scale = FOC_HAL_GetCurrentScale();

     s_config.current_calib.v_scale = FOC_HAL_GetVoltageScale();

 

     /* 清零上下文 */

     memset(&s_ctx, 0, sizeof(FOC_Context_t));

 

     /* 初始化 PID 控制器 */

     FOC_PID_Init(&s_ctx.pid_speed, &config->speed_pid);

     FOC_PID_Init(&s_ctx.pid_id,    &config->current_d_pid);

     FOC_PID_Init(&s_ctx.pid_iq,    &config->current_q_pid);

 

     /* 初始化观测器 */

     FOC_Observer_Init(&s_ctx);

     FOC_LastFault_Reset();

     FOC_Log_Reset();
     FOC_ResetCurrentAngleTrim();

 

     /* 初始化保护模块 */

     FOC_Protection_GetDefaultThreshold(&s_prot_threshold);

     FOC_Protection_SetThreshold(&s_prot_threshold);

 

     /* 默认 d 轴电流参考为 0（最大转矩电流比控制） */

     s_ctx.id_ref = 0.0f;

     s_ctx.iq_ref = 0.0f;
     s_foc_ctrl_source = FOC_CTRL_SOURCE_SPEED;

     /* 默认方向正转 */

     s_ctx.direction = FOC_DIR_CW;

 

     /* 进入待机态 */

     s_ctx.state = FOC_STATE_IDLE;

     s_ctx.fault = FOC_FAULT_NONE;

     for (motor = 1U; motor < FOC_CORE_MOTOR_COUNT; motor++) {
         FOC_Core_SelectMotor(motor);

         memcpy(&s_config, config, sizeof(FOC_Config_t));
         FOC_HAL_GetCurrentOffset(&s_config.current_calib);
         s_config.current_calib.i_scale = FOC_HAL_GetCurrentScale();
         s_config.current_calib.v_scale = FOC_HAL_GetVoltageScale();

         memset(&s_ctx, 0, sizeof(FOC_Context_t));

         FOC_PID_Init(&s_ctx.pid_speed, &config->speed_pid);
         FOC_PID_Init(&s_ctx.pid_id,    &config->current_d_pid);
         FOC_PID_Init(&s_ctx.pid_iq,    &config->current_q_pid);

         FOC_Observer_Init(&s_ctx);
         FOC_Log_Reset();
         FOC_ResetCurrentAngleTrim();

         s_ctx.id_ref = 0.0f;
         s_ctx.iq_ref = 0.0f;
         s_foc_ctrl_source = FOC_CTRL_SOURCE_SPEED;
         s_ctx.direction = FOC_DIR_CW;
         s_ctx.state = FOC_STATE_IDLE;
         s_ctx.fault = FOC_FAULT_NONE;
     }

     FOC_Core_SelectMotor(0U);

 

     return FOC_OK;

 }

 

 int FOC_Core_DeInit(void)

 {
     uint8_t motor;

     for (motor = 0U; motor < FOC_CORE_MOTOR_COUNT; motor++) {
         FOC_Core_SelectMotor(motor);

         if (s_ctx.state == FOC_STATE_RUNNING) {
             FOC_Core_Stop();
         }

         FOC_HAL_DisablePWM();
         memset(&s_ctx, 0, sizeof(FOC_Context_t));
         s_ctx.state = FOC_STATE_INIT;
     }

     FOC_Core_SelectMotor(0U);

     return FOC_OK;

 }

 

 /* ===================================================================

  *  启停控制

  * =================================================================== */

 

 int FOC_Core_Start(void)

 {

     if (s_ctx.state == FOC_STATE_FAULT) {

         return FOC_FAULT;

     }

 

     if (s_ctx.state != FOC_STATE_IDLE) {

         return FOC_BUSY;

     }

 

     /* 复位 PID，防止历史积分影响启动 */

     FOC_PID_Reset(&s_ctx.pid_speed);

     FOC_PID_Reset(&s_ctx.pid_id);

     FOC_PID_Reset(&s_ctx.pid_iq);
     FOC_ResetCurrentAngleTrim();
     s_speed_loop_accum_us = 0U;
     FOC_ResetSpeedRefRamp();
     FOC_ResetHallTravelStallGuard(1U);
     s_speed_error_boost_prev_ref = 0.0f;
     s_bidir_decel_hold_prev_ref = 0.0f;
     g_foc_bidir_decel_hold_active = 0U;
     g_foc_bidir_decel_hold_applied_mA = 0;
     g_foc_bidir_decel_hold_raw_err_rpm = 0;
     s_bidir_zero_soft_prev_ref = 0.0f;
     g_foc_bidir_zero_soft_active = 0U;
     g_foc_bidir_zero_soft_scale_percent = 100U;
     g_foc_bidir_zero_soft_limited_mA = 0;
     FOC_BidirZeroTransfer_Reset();



     /* 复位观测器 */

     FOC_Observer_Init(&s_ctx);

 

     /* 使能 PWM 输出 */

     FOC_Log_Reset();

     FOC_HAL_EnablePWM();

 

     /* 进入运行态 */

     s_ctx.state = FOC_STATE_RUNNING;

 

     return FOC_OK;

 }

 

 int FOC_Core_Stop(void)

 {

     /* 禁用 PWM 输出 */

     FOC_HAL_DisablePWM();

 

     /* 占空比归零 */

     s_ctx.duty_a = 0.0f;

     s_ctx.duty_b = 0.0f;

     s_ctx.duty_c = 0.0f;

     FOC_HAL_SetDutyCycle(0.0f, 0.0f, 0.0f);

 

     /* 复位 PID */

     FOC_PID_Reset(&s_ctx.pid_speed);

     FOC_PID_Reset(&s_ctx.pid_id);

     FOC_PID_Reset(&s_ctx.pid_iq);
     FOC_ResetCurrentAngleTrim();
     s_speed_loop_accum_us = 0U;
     s_speed_error_boost_prev_ref = 0.0f;

 

     /* 速度参考归零 */

     s_recovery_zero_vector_cycles = 0U;
     s_recovery_zero_vector_min_cycles = 0U;
     g_foc_recovery_zero_vector_remaining = 0U;
     g_foc_recovery_current_wait_active = 0U;

     s_ctx.speed_ref = 0.0f;
     FOC_ResetSpeedRefRamp();
     FOC_ResetHallTravelStallGuard(1U);
     g_foc_dyn_speed_enable = 0U;
     g_foc_bidir_speed_enable = 0U;
     s_dyn_speed_prev_enable = 0U;
     s_bidir_speed_prev_enable = 0U;
     g_foc_dyn_speed_reset_stats = 0U;
     g_foc_bidir_speed_reset_stats = 0U;

     s_ctx.iq_ref    = 0.0f;
     s_ctx.id_ref = 0.0f;
     s_foc_ctrl_source = FOC_CTRL_SOURCE_SPEED;
     s_ctx.speed_ctrl_fdb = 0.0f;
     g_foc_speed_ctrl_fdb_rpm = 0;
     s_ctx.speed_loop_counter = 0U;
     s_speed_loop_accum_us = 0U;
     s_speed_error_boost_prev_ref = 0.0f;
     s_bidir_decel_hold_prev_ref = 0.0f;
     g_foc_bidir_decel_hold_active = 0U;
     g_foc_bidir_decel_hold_applied_mA = 0;
     g_foc_bidir_decel_hold_raw_err_rpm = 0;
     s_bidir_zero_soft_prev_ref = 0.0f;
     g_foc_bidir_zero_soft_active = 0U;
     g_foc_bidir_zero_soft_scale_percent = 100U;
     g_foc_bidir_zero_soft_limited_mA = 0;
     FOC_BidirZeroTransfer_Reset();



     /* 进入待机态 */

     s_ctx.state = FOC_STATE_IDLE;

 

     return FOC_OK;

 }

 

 /* ===================================================================

  *  主控制循环

  * =================================================================== */

 

 static void FOC_Core_MainLoopOne(void)

 {
     uint32_t prof_enter_us;
     uint32_t prof_mark_us;
     uint32_t prof_next_us;
     uint32_t control_period_us;
     uint8_t control_period_late;
     uint8_t control_period_recovery;
     FOC_HallRaw_t hall_event_raw;
     uint32_t hall_event_timestamp_us = 0U;
     uint32_t hall_event_seq = 0U;
     uint32_t hall_sample_timestamp_us = 0U;
     uint8_t hall_timestamp_valid = 0U;

     prof_enter_us = FOC_Prof_Enter();
     control_period_us = FOC_ControlPeriodUs();
     control_period_late = FOC_ControlPeriodIsLate(control_period_us);
     (void)control_period_late;
     control_period_recovery = FOC_ControlPeriodNeedsRecovery(control_period_us);
     FOC_StateMachine();
     prof_mark_us = FOC_HAL_GetTimestampUs();
     g_foc_prof_after_state_us = prof_mark_us;
     FOC_Prof_RecordSegment(prof_enter_us, prof_mark_us, 1U);

 

     if (s_ctx.state != FOC_STATE_RUNNING) {

         FOC_Prof_Exit(prof_enter_us, prof_mark_us);
         return;

     }

 

     /* 预计算时间常量，避免热路径中的除法 */

     if (FOC_IsAutoTestMotor() != 0U) {
         FOC_DynSpeed_ServiceRef();
         FOC_BidirSpeed_ServiceRef();
     }

     FOC_NormalizeSignedSpeedRef();
     FOC_UpdateHallTravelStallGuard();

     float observer_dt = FOC_ControlDtFromUs(control_period_us,
                                             FOC_CONTROL_OBSERVER_DT_MAX_US);

     float pid_dt = FOC_ControlDtFromUs(control_period_us,
                                        FOC_CONTROL_PID_DT_MAX_US);

     float pid_inv_dt = FOC_ControlInvDtFromUs(control_period_us,
                                               FOC_CONTROL_PID_DT_MAX_US);

 

     /* ---- 1. 读取驱动层原始传感器数据 ---- */

     if ((FOC_HAL_GetHallEvent(&hall_event_raw,
                               &hall_event_timestamp_us,
                               &hall_event_seq) != 0U) &&
         (hall_event_seq != s_hall_event_seq_seen)) {
         s_ctx.hall_raw = hall_event_raw;
         s_hall_event_seq_seen = hall_event_seq;
         hall_sample_timestamp_us = hall_event_timestamp_us;
         hall_timestamp_valid = 1U;
         prof_next_us = FOC_HAL_GetTimestampUs();
         g_foc_hall_event_used_count++;
         g_foc_hall_event_seq = hall_event_seq;
         g_foc_hall_event_age_us = prof_next_us - hall_event_timestamp_us;
     } else {
         FOC_HAL_GetHallRaw(&s_ctx.hall_raw);
         prof_next_us = FOC_HAL_GetTimestampUs();
         hall_sample_timestamp_us = prof_next_us;
         hall_timestamp_valid = 0U;
         g_foc_hall_poll_count++;
     }
     g_foc_prof_after_hall_us = prof_next_us;
     FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 2U);
     prof_mark_us = prof_next_us;

     FOC_HAL_GetPhaseCurrentsRaw(&s_ctx.i_abc_raw);

     s_ctx.v_bus_raw  = FOC_HAL_GetBusVoltageRaw();
     prof_next_us = FOC_HAL_GetTimestampUs();
     g_foc_prof_after_adc_us = prof_next_us;
     FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 3U);
     prof_mark_us = prof_next_us;

 

     /* ---- 2. 原始数据解析（观测器负责物理量换算） ---- */

     FOC_HallSector_t hall_candidate;

     FOC_Observer_HallRawToSector(&s_ctx.hall_raw, &hall_candidate);
     FOC_ApplyHallSector(&hall_candidate,
                         hall_sample_timestamp_us,
                         hall_timestamp_valid,
                         control_period_recovery);

     if (s_ctx.fault != FOC_FAULT_NONE) {
         FOC_EnterFaultState();
         FOC_Log_Record(s_ctx.theta_e_predicted);
         prof_next_us = FOC_HAL_GetTimestampUs();
         FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 4U);
         FOC_Prof_Exit(prof_enter_us, prof_next_us);
         return;
     }

 

     /* 有效扇区：更新电角度基准；无效扇区：保留上一次角度 */

     if (s_ctx.hall_sector.sector != 0U) {

         s_ctx.theta_e = s_ctx.hall_sector.theta_e;

         s_ctx.theta_m = s_ctx.theta_e / (float)s_config.motor.pole_pairs;

     }

     FOC_Observer_ParsePhaseCurrents(&s_ctx.i_abc_raw, &s_config.current_calib, &s_ctx.i_abc);
     s_ctx.v_bus = FOC_Observer_ParseBusVoltage(s_ctx.v_bus_raw, &s_config.current_calib);
     g_foc_vbus_mV = FOC_Log_ToU16(s_ctx.v_bus, 1000.0f);

     if (control_period_recovery != 0U) {
         float theta_recovery;

         if (s_ctx.hall_sector.sector != 0U) {
             s_ctx.speed_fdb = FOC_Observer_CalcSpeed(&s_ctx, s_ctx.theta_e,
                                                       observer_dt,
                                                       s_config.motor.pole_pairs);
             s_ctx.theta_e_predicted = s_ctx.theta_e;
             s_hall_recovery_accept_cycles = FOC_HALL_RECOVERY_ACCEPT_CYCLES;
         }

         FOC_ResetClosedLoopForRecovery();
         FOC_BeginRecoveryZeroVectorHold();
         FOC_BeginPostRecoveryDutySlew();

         FOC_Protection_Check(&s_ctx, s_ctx.v_bus);
         if (s_ctx.fault != FOC_FAULT_NONE) {
             FOC_EnterFaultState();
         } else {
             FOC_ServiceRecoveryZeroVectorHold();
         }

         theta_recovery = s_ctx.theta_e_predicted;
         FOC_Clarke(&s_ctx.i_abc, &s_ctx.i_ab);
         FOC_Park(&s_ctx.i_ab, theta_recovery, &s_ctx.i_dq);
         if (s_ctx.fault == FOC_FAULT_NONE) {
             FOC_RunRecoveryCurrentControl(theta_recovery, pid_dt, pid_inv_dt);
         }
         FOC_Log_Record(theta_recovery);

         prof_next_us = FOC_HAL_GetTimestampUs();
         FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 4U);
         FOC_Prof_Exit(prof_enter_us, prof_next_us);
         return;
     }

 

     /* 角度预测：必须在 CalcSpeed 之前调用！

      * CalcSpeed 会更新 hall_sector_prev，若先调 CalcSpeed 则

      * PredictAngle 检测不到扇区跳变，无法同步角度 */

     if (s_recovery_zero_vector_cycles > 0U) {
         float theta_recovery = FOC_Observer_PredictAngle(&s_ctx, observer_dt,
                                                          s_config.motor.pole_pairs);

         s_ctx.speed_fdb = FOC_Observer_CalcSpeed(&s_ctx, s_ctx.theta_e,
                                                   observer_dt,
                                                   s_config.motor.pole_pairs);

         if (s_ctx.sector_no_change_count >= FOC_SECTOR_NO_CHANGE_THRESHOLD) {
             theta_recovery = s_ctx.theta_e_predicted;
         }

         FOC_PID_Reset(&s_ctx.pid_speed);
         s_ctx.iq_ref = 0.0f;
         s_ctx.speed_ctrl_fdb = 0.0f;
         g_foc_speed_ctrl_fdb_rpm = 0;
         s_ctx.speed_loop_counter = 0U;

         FOC_Protection_Check(&s_ctx, s_ctx.v_bus);
         if (s_ctx.fault != FOC_FAULT_NONE) {
             FOC_EnterFaultState();
         } else {
             FOC_ServiceRecoveryZeroVectorHold();
         }

         FOC_Clarke(&s_ctx.i_abc, &s_ctx.i_ab);
         FOC_Park(&s_ctx.i_ab, theta_recovery, &s_ctx.i_dq);
         if (s_ctx.fault == FOC_FAULT_NONE) {
             FOC_RunRecoveryCurrentControl(theta_recovery, pid_dt, pid_inv_dt);
         }
         FOC_Log_Record(theta_recovery);

         prof_next_us = FOC_HAL_GetTimestampUs();
         FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 4U);
         FOC_Prof_Exit(prof_enter_us, prof_next_us);
         return;
     }

     FOC_UpdateSpeedRefRamp(control_period_us);
     if ((g_foc_bidir_speed_enable != 0U) &&
         (s_bidir_zero_state == FOC_BIDIR_ZERO_STATE_HOLD) &&
         (FOC_FABS(s_ctx.speed_ref) < 0.5f)) {
         FOC_ResetSpeedLoopForZeroHold();
     }

     float theta_e_ctrl = FOC_Observer_PredictAngle(&s_ctx, observer_dt,

                                                     s_config.motor.pole_pairs);

 

     s_ctx.speed_fdb = FOC_Observer_CalcSpeed(&s_ctx, s_ctx.theta_e,

                                               observer_dt, s_config.motor.pole_pairs);
     FOC_UpdateSpeedControlFeedback();

     if (s_ctx.sector_no_change_count >= FOC_SECTOR_NO_CHANGE_THRESHOLD) {
         theta_e_ctrl = s_ctx.theta_e_predicted;
     }

 


 

     theta_e_ctrl = FOC_ApplyCurrentAngleTrim(theta_e_ctrl);

 

     /* ---- 3. Clarke 变换 ---- */

     FOC_Clarke(&s_ctx.i_abc, &s_ctx.i_ab);

 

     /* ---- 4. Park 变换 ---- */

     FOC_Park(&s_ctx.i_ab, theta_e_ctrl, &s_ctx.i_dq);
     FOC_UpdateCurrentAngleTrim(pid_dt);
     FOC_CheckSpeedFdbDropFault();
     FOC_CheckSpeedDropFault();

     /* ---- 5. 保护检测：必须早于 PID / SVPWM / PWM 输出 ---- */

     FOC_Protection_Check(&s_ctx, s_ctx.v_bus);
     if (s_ctx.fault != FOC_FAULT_NONE) {
         FOC_EnterFaultState();
         FOC_Log_Record(theta_e_ctrl);
         prof_next_us = FOC_HAL_GetTimestampUs();
         FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 4U);
         FOC_Prof_Exit(prof_enter_us, prof_next_us);
         return;
     }

 

     /* ---- 6. 速度环 PID (降采样) ---- */

     {
         uint32_t speed_step_us = control_period_us;
         uint32_t speed_loop_period_us =
             FOC_CONTROL_PERIOD_US * (uint32_t)FOC_SPEED_LOOP_DOWNSAMPLE;

         if (speed_loop_period_us == 0U) {
             speed_loop_period_us = FOC_CONTROL_PERIOD_US;
         }
         if (speed_step_us == 0U) {
             speed_step_us = FOC_CONTROL_PERIOD_US;
         }
         if (speed_step_us > FOC_CONTROL_PID_DT_MAX_US) {
             speed_step_us = FOC_CONTROL_PID_DT_MAX_US;
         }

         if ((0xFFFFFFFFU - s_speed_loop_accum_us) >= speed_step_us) {
             s_speed_loop_accum_us += speed_step_us;
         } else {
             s_speed_loop_accum_us = speed_loop_period_us;
         }

         if (s_ctx.speed_loop_counter < 65535U) {
             s_ctx.speed_loop_counter++;
         }

         if ((s_foc_ctrl_source == FOC_CTRL_SOURCE_SPEED) &&
             (s_speed_loop_accum_us >= speed_loop_period_us)) {
             float speed_dt = (float)s_speed_loop_accum_us * 1.0e-6f;
             float speed_inv_dt = 1000000.0f / (float)s_speed_loop_accum_us;
             float speed_ref_ctrl = s_speed_ref_ctrl;
             float speed_error = speed_ref_ctrl - s_ctx.speed_ctrl_fdb;
             float speed_iq_ref_max =
                 FOC_GetSpeedIqPositiveLimit(speed_ref_ctrl, speed_error);
             float speed_iq_ref_max_saved = s_ctx.pid_speed.out_max;
             float speed_iq_ref;
             uint8_t zero_pid_frozen = FOC_BidirZeroTransfer_PidFrozen();
             uint8_t hall_travel_stall_blocked =
                 FOC_HallTravelStallTorqueBlocked();

             s_speed_loop_accum_us = 0U;
             s_ctx.speed_loop_counter = 0U;

             if (hall_travel_stall_blocked != 0U) {
                 speed_iq_ref = 0.0f;
                 FOC_PID_Reset(&s_ctx.pid_speed);
                 g_foc_speed_error_boost_mA = 0;
             } else if (zero_pid_frozen != 0U) {
                 speed_iq_ref = s_ctx.iq_ref;
                 FOC_PID_Reset(&s_ctx.pid_speed);
                 g_foc_speed_error_boost_mA = 0;
             } else {
                 s_ctx.pid_speed.out_max = speed_iq_ref_max;
                 speed_iq_ref = FOC_PID_Update(&s_ctx.pid_speed,
                                                speed_error,
                                                speed_dt, speed_inv_dt);
                 s_ctx.pid_speed.out_max = speed_iq_ref_max_saved;

#if FOC_SPEED_ERROR_BOOST_ENABLE
                 if ((speed_ref_ctrl >= FOC_SPEED_ERROR_BOOST_MIN_RPM) &&
                     (speed_error > FOC_SPEED_ERROR_BOOST_DEADBAND_RPM) &&
                     ((speed_ref_ctrl - s_ctx.speed_fdb) >
                      FOC_SPEED_ERROR_BOOST_DEADBAND_RPM) &&
                     (speed_ref_ctrl >
                      (s_speed_error_boost_prev_ref +
                       FOC_SPEED_ERROR_BOOST_REF_RISE_MIN_RPM))) {
                     float boost =
                         FOC_SPEED_ERROR_BOOST_KP *
                         (speed_error - FOC_SPEED_ERROR_BOOST_DEADBAND_RPM);

                     boost = FOC_CLAMP(boost, 0.0f,
                                       FOC_SPEED_ERROR_BOOST_MAX_A);

                     speed_iq_ref += boost;
                     g_foc_speed_error_boost_mA =
                         FOC_Log_ToI16(boost, 1000.0f);
                 } else {
                     g_foc_speed_error_boost_mA = 0;
                 }
#else
                 g_foc_speed_error_boost_mA = 0;
#endif
             }
             if ((hall_travel_stall_blocked == 0U) &&
                 (zero_pid_frozen == 0U) &&
                 (g_foc_bidir_speed_enable != 0U) &&
                 (g_foc_bidir_zero_cross_state != FOC_BIDIR_ZERO_STATE_HOLD) &&
                 (g_foc_bidir_zero_approach_active != 0U)) {
                 float brake_limit =
                     (float)g_foc_bidir_zero_approach_brake_limit_mA;

                 if (g_foc_bidir_zero_tail_active != 0U) {
                     brake_limit =
                         (float)g_foc_bidir_zero_tail_brake_limit_mA;
                 }
                 brake_limit *= 0.001f;

                 if (brake_limit < 0.0f) {
                     brake_limit = -brake_limit;
                 }
                 if (speed_iq_ref < -brake_limit) {
                     speed_iq_ref = -brake_limit;
                     if (g_foc_bidir_zero_brake_limited_count < 0xFFFFFFFFU) {
                         g_foc_bidir_zero_brake_limited_count++;
                     }
                 }
             }
             s_speed_error_boost_prev_ref = speed_ref_ctrl;

             if ((hall_travel_stall_blocked == 0U) &&
                 (zero_pid_frozen == 0U)) {
                 speed_iq_ref = FOC_ApplyLowSpeedTorqueAssist(speed_iq_ref,
                                                              speed_ref_ctrl,
                                                              speed_error);
                 speed_iq_ref = FOC_ApplyBidirTailDriveAssist(speed_iq_ref,
                                                              speed_ref_ctrl,
                                                              speed_error);
                 speed_iq_ref = FOC_ApplyBidirDecelHoldAssist(speed_iq_ref,
                                                              speed_ref_ctrl);
                 speed_iq_ref = FOC_ApplyBidirZeroSoftLanding(speed_iq_ref,
                                                              speed_ref_ctrl);
             } else {
                 g_foc_low_speed_torque_active = 0U;
                 g_foc_low_speed_torque_applied_mA = 0;
                 g_foc_bidir_decel_hold_active = 0U;
                 g_foc_bidir_decel_hold_applied_mA = 0;
                 g_foc_bidir_decel_hold_raw_err_rpm = 0;
                 g_foc_bidir_zero_soft_active = 0U;
                 g_foc_bidir_zero_soft_scale_percent = 100U;
                 g_foc_bidir_zero_soft_limited_mA = 0;
                 g_foc_lift_current_limit_active = 0U;
                 g_foc_lift_current_limit_extra_mA = 0;
             }
             speed_iq_ref = FOC_BidirZeroTransfer_ApplyIq(speed_iq_ref,
                                                          speed_dt);
             if ((hall_travel_stall_blocked == 0U) &&
                 (zero_pid_frozen == 0U)) {
                 speed_iq_ref = FOC_ApplyLowSpeedIqSlew(speed_iq_ref,
                                                        speed_ref_ctrl,
                                                        speed_dt);
             } else {
                 g_foc_low_speed_iq_slew_active = 0U;
                 g_foc_low_speed_iq_slew_limited_mA = 0;
             }
             speed_iq_ref = FOC_LimitRegenBrakingIq(speed_iq_ref);
             s_ctx.iq_ref = FOC_CLAMP(speed_iq_ref,
                                      s_ctx.pid_speed.out_min,
                                      speed_iq_ref_max);

         }
     }

 

     /* ---- 7. 电流环 PID ---- */

     if ((g_foc_bidir_speed_enable != 0U) &&
         (s_bidir_zero_state == FOC_BIDIR_ZERO_STATE_HOLD) &&
         (FOC_FABS(s_ctx.speed_ref) < 0.5f)) {
         FOC_ResetSpeedLoopForZeroHold();
     }

     if (s_foc_ctrl_source != FOC_CTRL_SOURCE_SPEED) {
         s_speed_loop_accum_us = 0U;
         s_ctx.speed_loop_counter = 0U;
         g_foc_speed_error_boost_mA = 0;
         s_speed_error_boost_prev_ref = 0.0f;
         g_foc_low_speed_torque_active = 0U;
         g_foc_low_speed_torque_applied_mA = 0;
         g_foc_low_speed_iq_slew_active = 0U;
         g_foc_low_speed_iq_slew_limited_mA = 0;
         g_foc_lift_current_limit_active = 0U;
         g_foc_lift_current_limit_extra_mA = 0;
         g_foc_bidir_decel_hold_active = 0U;
         g_foc_bidir_decel_hold_applied_mA = 0;
         g_foc_bidir_decel_hold_raw_err_rpm = 0;
         g_foc_bidir_zero_soft_active = 0U;
         g_foc_bidir_zero_soft_scale_percent = 100U;
         g_foc_bidir_zero_soft_limited_mA = 0;
         g_foc_current_q_ff_mV = 0;
     }

     s_ctx.v_dq.d = FOC_PID_Update(&s_ctx.pid_id,

                                     s_ctx.id_ref - s_ctx.i_dq.d,

                                     pid_dt, pid_inv_dt);

     s_ctx.v_dq.q = FOC_PID_Update(&s_ctx.pid_iq,

                                     FOC_ControlIqRef() - s_ctx.i_dq.q,

                                     pid_dt, pid_inv_dt);
     s_ctx.v_dq.q = FOC_ApplyLowSpeedCurrentFeedForward(s_ctx.v_dq.q);

 

     /* ---- 8. 逆 Park 变换 ---- */

     FOC_InvPark(&s_ctx.v_dq, theta_e_ctrl, &s_ctx.v_ab);

 

     /* ---- 9. SVPWM 调制 ---- */

     float duty_prev_a = s_ctx.duty_a;
     float duty_prev_b = s_ctx.duty_b;
     float duty_prev_c = s_ctx.duty_c;

     FOC_SVPWM_Calculate(s_ctx.v_ab.alpha, s_ctx.v_ab.beta, s_ctx.v_bus,

                          &s_ctx.duty_a, &s_ctx.duty_b, &s_ctx.duty_c);
     FOC_ApplyPostRecoveryDutySlew(duty_prev_a, duty_prev_b, duty_prev_c);
     prof_next_us = FOC_HAL_GetTimestampUs();
     g_foc_prof_after_calc_us = prof_next_us;
     FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 4U);
     prof_mark_us = prof_next_us;

 

     /* ---- 10. 输出 PWM 占空比 ---- */

     FOC_HAL_SetDutyCycle(s_ctx.duty_a, s_ctx.duty_b, s_ctx.duty_c);
     prof_next_us = FOC_HAL_GetTimestampUs();
     g_foc_prof_after_pwm_us = prof_next_us;
     FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 5U);
     prof_mark_us = prof_next_us;

 

     if (FOC_IsAutoTestMotor() != 0U) {
         FOC_DynSpeed_ServiceMetrics();
     }
     FOC_DetailLog_Service(theta_e_ctrl, prof_next_us);
     FOC_StartLog_Service(prof_next_us);
     prof_next_us = FOC_HAL_GetTimestampUs();
     FOC_Prof_RecordSegment(prof_mark_us, prof_next_us, 6U);
     FOC_Prof_Exit(prof_enter_us, prof_next_us);

 }

 

 /* ===================================================================

  *  状态机

  * =================================================================== */

 

 static void FOC_StateMachine(void)

 {

     switch (s_ctx.state) {

         case FOC_STATE_INIT:

             /* 初始化完成后应已转入 IDLE，不应停留于此 */

             break;

 

         case FOC_STATE_IDLE:

             /* 待机态，等待 FOC_Core_Start() 启动 */

             break;

 

         case FOC_STATE_RUNNING:

             /* 检测到故障 → 转入 FAULT 态 */

             if (s_ctx.fault != FOC_FAULT_NONE) {

                 FOC_EnterFaultState();

             }

             break;

 

         case FOC_STATE_FAULT:

             /* 故障态，PWM 已关闭，等待 FOC_Core_ClearFault() */

             break;

 

         default:

             s_ctx.state = FOC_STATE_IDLE;

             break;

     }

 }

 

 /* ===================================================================

  *  运行时参数设置（主线程调用，需临界区保护共享变量）

  * =================================================================== */

 

 void FOC_Core_MainLoop(void)
 {
     uint8_t previous_motor = FOC_Core_GetSelectedMotor();
     uint8_t motor;

     for (motor = 0U; motor < FOC_CORE_MOTOR_COUNT; motor++) {
         FOC_Core_SelectMotor(motor);
         FOC_Core_MainLoopOne();
     }

     FOC_Core_SelectMotor(previous_motor);
 }

 const FOC_Context_t *FOC_Core_GetContext(void)

 {

     return &s_ctx;

 }

 int FOC_Core_ReadHallTravel(uint8_t motor_id, int16_t *hall_states_offset)
 {
     int32_t travel;

     if ((motor_id >= FOC_CORE_MOTOR_COUNT) ||
         (hall_states_offset == NULL)) {
         return FOC_ERR;
     }

     FOC_HAL_EnterCritical();
     travel = g_foc_hall_travel_count[motor_id] +
              (int32_t)g_foc_hall_travel_offset[motor_id];
     *hall_states_offset = FOC_ClampI32ToI16(travel);
     FOC_HAL_ExitCritical();

     return FOC_OK;
 }

int FOC_Core_ReadHallStats(uint8_t motor_id,
                           int64_t *hall_distance,
                           uint64_t *total_hall_counts,
                           uint64_t *current_direction_hall_counts)
{
     if ((motor_id >= FOC_CORE_MOTOR_COUNT) ||
         (hall_distance == NULL) ||
         (total_hall_counts == NULL) ||
         (current_direction_hall_counts == NULL)) {
         return FOC_ERR;
     }

     FOC_HAL_EnterCritical();
     *hall_distance =
         (int64_t)g_foc_hall_travel_count[motor_id] +
         (int64_t)g_foc_hall_travel_offset[motor_id];
     *total_hall_counts = s_foc_hall_total_update_count[motor_id];
     *current_direction_hall_counts =
         s_foc_hall_direction_update_count[motor_id];
     FOC_HAL_ExitCritical();

     return FOC_OK;
}

int FOC_Core_CopyHallHistory(uint8_t motor_id,
                             uint32_t *delta_time_us,
                             uint8_t *history_hall,
                             uint64_t *hall_counts_history,
                             uint16_t history_size,
                             int16_t *head_index_hall,
                             int16_t *head_index_app_hall)
{
     uint16_t i;
     uint16_t copy_count;

     if ((motor_id >= FOC_CORE_MOTOR_COUNT) ||
         (delta_time_us == NULL) ||
         (history_hall == NULL) ||
         (hall_counts_history == NULL) ||
         (head_index_hall == NULL) ||
         (head_index_app_hall == NULL) ||
         (history_size == 0U)) {
         return FOC_ERR;
     }

     copy_count = (history_size < FOC_HALL_HISTORY_SIZE)
                ? history_size
                : FOC_HALL_HISTORY_SIZE;

     FOC_HAL_EnterCritical();
     for (i = 0U; i < copy_count; i++) {
         delta_time_us[i] = s_foc_hall_history_delta_us[motor_id][i];
         history_hall[i] = s_foc_hall_history_raw[motor_id][i];
         hall_counts_history[i] = s_foc_hall_history_count[motor_id][i];
     }
     *head_index_hall = (s_foc_hall_history_valid_count[motor_id] == 0U)
                      ? -1
                      : s_foc_hall_history_head[motor_id];
     *head_index_app_hall = *head_index_hall;
     FOC_HAL_ExitCritical();

     for (i = copy_count; i < history_size; i++) {
         delta_time_us[i] = 0U;
         history_hall[i] = 0U;
         hall_counts_history[i] = 0U;
     }

     return FOC_OK;
}

int FOC_Core_WriteHallTravelOffset(uint8_t motor_id,
                                   int16_t *hall_states_offset,
                                   int16_t hall_position)
{
     if ((motor_id >= FOC_CORE_MOTOR_COUNT) ||
         (hall_states_offset == NULL)) {
         return FOC_ERR;
     }

     FOC_HAL_EnterCritical();
     g_foc_hall_travel_offset[motor_id] =
         (int32_t)hall_position - g_foc_hall_travel_count[motor_id];
     s_foc_hall_travel_stall_active_store[motor_id] = 0U;
     s_foc_hall_travel_stall_counter_store[motor_id] = 0U;
     s_foc_hall_travel_stall_dir_store[motor_id] = 0;
     s_foc_hall_travel_suppressed_delta_store[motor_id] = 0;
     g_foc_hall_travel_stall_active[motor_id] = 0U;
     g_foc_hall_travel_stall_counter[motor_id] = 0U;
     g_foc_hall_travel_stall_dir[motor_id] = 0;
     g_foc_hall_travel_suppressed_delta[motor_id] = 0;
     g_foc_hall_travel_freeze_count[motor_id] = 0U;
     *hall_states_offset = hall_position;
     FOC_HAL_ExitCritical();

     return FOC_OK;
}

 

int FOC_Core_SetSpeedRef(float rpm)

 {

     FOC_HAL_EnterCritical();
     s_foc_ctrl_source = FOC_CTRL_SOURCE_SPEED;
     s_ctx.id_ref = 0.0f;
     FOC_HAL_ExitCritical();

     /* 限幅到电机最大转速 */

     rpm = FOC_ApplyAppDirectionInvertToRef(rpm);

     if (rpm > s_config.motor.max_speed_rpm) {

         rpm = s_config.motor.max_speed_rpm;

     } else if (rpm < -s_config.motor.max_speed_rpm) {

         rpm = -s_config.motor.max_speed_rpm;

     }

 

     /* 临界区保护：speed_ref 被 ISR 中的主循环读取 */

     FOC_HAL_EnterCritical();

     s_ctx.speed_ref = rpm;

 

    /* 非零速度更新方向；0rpm 保持当前方向并直接给 0rpm 控制目标 */

     if (rpm > 0.0f) {

         s_ctx.direction = FOC_DIR_CW;

     } else if (rpm < 0.0f) {

         s_ctx.direction = FOC_DIR_CCW;

         s_ctx.speed_ref = -rpm; /* 速度绝对值用于PID，方向由电角度处理 */

     } else {

         s_ctx.speed_ref = 0.0f;

     }

     FOC_HAL_ExitCritical();

 

     return FOC_OK;

 }

 

 int FOC_Core_SetCurrentRef(float id, float iq)

{

     float max_current = s_config.motor.max_current_a;

     if (max_current <= 0.0f) {
         return FOC_ERR;
     }

     iq = FOC_ApplyAppDirectionInvertToRef(iq);

     id = FOC_CLAMP(id, -max_current, max_current);
     iq = FOC_CLAMP(iq, -max_current, max_current);

     FOC_HAL_EnterCritical();

     s_foc_ctrl_source = FOC_CTRL_SOURCE_CURRENT;
     s_ctx.speed_ref = 0.0f;
     s_ctx.speed_ref_ctrl = 0.0f;
     s_speed_ref_ctrl = 0.0f;
     s_speed_ref_ctrl_direction = (iq < 0.0f) ? FOC_DIR_CCW : FOC_DIR_CW;
     g_foc_speed_ref_cmd_rpm = 0;
     g_foc_speed_ref_ctrl_rpm = 0;
     g_foc_speed_ref_ramp_active = 0U;

     g_foc_dyn_speed_enable = 0U;
     g_foc_bidir_speed_enable = 0U;
     s_dyn_speed_prev_enable = 0U;
     s_bidir_speed_prev_enable = 0U;
     g_foc_dyn_speed_reset_stats = 0U;
     g_foc_bidir_speed_reset_stats = 0U;

     FOC_PID_Reset(&s_ctx.pid_speed);
     s_speed_loop_accum_us = 0U;
     s_ctx.speed_loop_counter = 0U;
     s_ctx.speed_ctrl_fdb = 0.0f;
     g_foc_speed_ctrl_fdb_rpm = 0;
     g_foc_speed_error_boost_mA = 0;
     s_speed_error_boost_prev_ref = 0.0f;
     s_bidir_decel_hold_prev_ref = 0.0f;
     g_foc_bidir_decel_hold_active = 0U;
     g_foc_bidir_decel_hold_applied_mA = 0;
     g_foc_bidir_decel_hold_raw_err_rpm = 0;
     s_bidir_zero_soft_prev_ref = 0.0f;
     g_foc_bidir_zero_soft_active = 0U;
     g_foc_bidir_zero_soft_scale_percent = 100U;
     g_foc_bidir_zero_soft_limited_mA = 0;
     g_foc_lift_current_limit_active = 0U;
     g_foc_lift_current_limit_extra_mA = 0;
     FOC_BidirZeroTransfer_Reset();

     s_ctx.id_ref = id;
     if (iq < 0.0f) {
         s_ctx.direction = FOC_DIR_CCW;
         s_ctx.iq_ref = -iq;
     } else {
         s_ctx.direction = FOC_DIR_CW;
         s_ctx.iq_ref = iq;
     }

     FOC_HAL_ExitCritical();

     return FOC_OK;

}
int FOC_Core_SetDirection(FOC_Dir_e dir)

 {

     FOC_HAL_EnterCritical();

     dir = FOC_ApplyAppDirectionInvertToDir(dir);

     s_ctx.direction = dir;

     FOC_HAL_ExitCritical();

     return FOC_OK;

 }

 

 int FOC_Core_SetSpeedPID(const FOC_PID_Params_t *pid)

 {

     if (pid == NULL) return FOC_ERR;

     FOC_HAL_EnterCritical();

     FOC_PID_SetParams(&s_ctx.pid_speed, pid);

     FOC_HAL_ExitCritical();

     return FOC_OK;

 }

 

 int FOC_Core_SetCurrentPID(const FOC_PID_Params_t *pid_d, const FOC_PID_Params_t *pid_q)

 {

     if (pid_d == NULL || pid_q == NULL) return FOC_ERR;

     FOC_HAL_EnterCritical();

     FOC_PID_SetParams(&s_ctx.pid_id, pid_d);

     FOC_PID_SetParams(&s_ctx.pid_iq, pid_q);

     FOC_HAL_ExitCritical();

     return FOC_OK;

 }

 

 int FOC_Core_ClearFault(void)

 {

     if (s_ctx.state != FOC_STATE_FAULT) {

         return FOC_OK;

     }

 

     FOC_Protection_ClearFault(&s_ctx);
     FOC_ResetSpeedDropFaultMonitor();
     FOC_ResetSpeedFdbDropFaultMonitor();
     s_hall_illegal_transition_count = 0U;

     s_ctx.state = FOC_STATE_IDLE;

 

     return FOC_OK;

 }

 


