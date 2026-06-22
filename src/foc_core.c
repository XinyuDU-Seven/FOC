
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

FOC_Context_t s_ctx;

 

 /** 模块配置缓存 */

FOC_Config_t  s_config;

 

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
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_dir_reject_count = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_dir_reject_prev_sector = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_dir_reject_cur_sector = 0U;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_hall_dir_reject_direction = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_current_angle_trim_mrad = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_ccw_angle_offset_mrad =
    FOC_CCW_CONTROL_ANGLE_OFFSET_MRAD;
FOC_DEBUG_ROOT volatile int16_t  g_foc_control_angle_offset_mrad = 0;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_event_used_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_event_seq = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_event_age_us = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_hall_poll_count = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_ctrl_fdb_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_error_boost_mA = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_ref_cmd_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_speed_ref_ctrl_rpm = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_speed_ref_ramp_active = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_signed_speed_ref_normalize_count = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_vbus_mV = 0U;
FOC_DEBUG_ROOT volatile uint16_t g_foc_last_fault_vbus_mV = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_vbus_brake_limit_mA = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_vbus_brake_active = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_vbus_brake_limited_count = 0U;
FOC_DEBUG_ROOT volatile uint32_t g_foc_dyn_core_loop_count = 0U;
FOC_DEBUG_ROOT volatile int16_t  g_foc_dyn_core_seen_ref_rpm = 0;
FOC_DEBUG_ROOT volatile int16_t  g_foc_dyn_core_seen_fdb_rpm = 0;
FOC_DEBUG_ROOT volatile uint8_t  g_foc_dyn_core_state = 0U;

#define FOC_DYN_SPEED_LOG_SIZE 128U

extern volatile uint8_t  g_foc_dyn_speed_enable;
extern volatile uint8_t  g_foc_dyn_speed_reverse;
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
extern volatile uint8_t  g_foc_bidir_speed_enable;
extern volatile uint8_t  g_foc_bidir_speed_reset_stats;
extern volatile uint32_t g_foc_bidir_speed_period_ms;
extern volatile uint16_t g_foc_bidir_speed_max_rpm;
extern volatile uint32_t g_foc_bidir_speed_elapsed_ms;
extern volatile uint16_t g_foc_bidir_speed_phase_u16;
extern volatile int16_t  g_foc_bidir_speed_ref_rpm;

static uint16_t s_foc_log_decim = 0U;
static uint16_t s_hall_illegal_transition_count = 0U;
static uint32_t s_foc_prof_last_enter_us = 0U;
static uint32_t s_foc_control_period_us = FOC_CONTROL_PERIOD_US;
static uint16_t s_hall_recovery_accept_cycles = 0U;
static uint16_t s_post_recovery_duty_slew_cycles = 0U;
static uint16_t s_recovery_zero_vector_cycles = 0U;
static uint16_t s_recovery_zero_vector_min_cycles = 0U;
static uint32_t s_speed_loop_accum_us = 0U;
static float s_speed_ref_ctrl = 0.0f;
static FOC_Dir_e s_speed_ref_ctrl_direction = FOC_DIR_CW;
static float s_speed_error_boost_prev_ref = 0.0f;
static float s_current_angle_trim_rad = 0.0f;
static uint32_t s_hall_event_seq_seen = 0U;
static uint8_t s_dyn_speed_prev_enable = 0U;
static uint32_t s_dyn_speed_start_us = 0U;
static uint32_t s_dyn_log_last_us = 0U;
static float s_dyn_abs_err_avg_rpm = 0.0f;
static uint8_t s_bidir_speed_prev_enable = 0U;
static uint32_t s_bidir_speed_start_us = 0U;

 

 /* ===================================================================

  *  内部函数声明

  * =================================================================== */

 

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
static void FOC_ResetCurrentAngleTrim(void);
static float FOC_ApplyCurrentAngleTrim(float theta_ctrl);
static void FOC_UpdateCurrentAngleTrim(float dt);
static void FOC_UpdateSpeedControlFeedback(uint32_t dt_us);
static void FOC_DynSpeed_ServiceRef(void);
static void FOC_BidirSpeed_ServiceRef(void);
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
 static uint8_t FOC_HallStepMatchesControlDirection(uint8_t from, uint8_t to);
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

 static float FOC_ControlIqRef(void)
 {
     return (s_ctx.direction == FOC_DIR_CCW) ? -s_ctx.iq_ref : s_ctx.iq_ref;
 }
 static float FOC_SignedSpeedRef(void)
 {
     float ref = FOC_FABS(s_ctx.speed_ref);

     return (s_ctx.direction == FOC_DIR_CCW) ? -ref : ref;
 }
 static void FOC_ResetSpeedRefRamp(void)
 {
     s_speed_ref_ctrl = 0.0f;
     s_speed_ref_ctrl_direction = s_ctx.direction;
     g_foc_speed_ref_cmd_rpm = FOC_Log_ToI16(FOC_FABS(s_ctx.speed_ref), 1.0f);
     g_foc_speed_ref_ctrl_rpm = 0;
     g_foc_speed_ref_ramp_active = 0U;
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
     float rate;
     float step;
     float delta;

     if (target > s_config.motor.max_speed_rpm) {
         target = s_config.motor.max_speed_rpm;
     }

     if (s_speed_ref_ctrl_direction != s_ctx.direction) {
         s_speed_ref_ctrl = 0.0f;
         s_speed_ref_ctrl_direction = s_ctx.direction;
         FOC_PID_Reset(&s_ctx.pid_speed);
         FOC_PID_Reset(&s_ctx.pid_iq);
         s_ctx.iq_ref = 0.0f;
         s_speed_loop_accum_us = 0U;
         s_speed_error_boost_prev_ref = 0.0f;
     }

     if (dt_us == 0U) {
         dt_us = FOC_CONTROL_PERIOD_US;
     } else if (dt_us > FOC_CONTROL_PID_DT_MAX_US) {
         dt_us = FOC_CONTROL_PID_DT_MAX_US;
     }

     if (target > s_speed_ref_ctrl) {
         rate = FOC_SPEED_REF_RAMP_UP_RPM_PER_S;
         delta = target - s_speed_ref_ctrl;
     } else {
         rate = FOC_SPEED_REF_RAMP_DOWN_RPM_PER_S;
         delta = s_speed_ref_ctrl - target;
     }

     if (rate <= 0.0f) {
         s_speed_ref_ctrl = target;
     } else if (delta > 0.0f) {
         step = rate * ((float)dt_us * 1.0e-6f);
         if (step >= delta) {
             s_speed_ref_ctrl = target;
         } else if (target > s_speed_ref_ctrl) {
             s_speed_ref_ctrl += step;
         } else {
             s_speed_ref_ctrl -= step;
         }
     }

     g_foc_speed_ref_cmd_rpm = FOC_Log_ToI16(target, 1.0f);
     g_foc_speed_ref_ctrl_rpm = FOC_Log_ToI16(s_speed_ref_ctrl, 1.0f);
     g_foc_speed_ref_ramp_active =
         (FOC_FABS(target - s_speed_ref_ctrl) > 0.5f) ? 1U : 0U;

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
    s_dyn_abs_err_avg_rpm = 0.0f;
    s_dyn_speed_start_us = now_us;
    FOC_DynSpeed_ResetLog();
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

    g_foc_dyn_speed_elapsed_ms = elapsed_us / 1000U;
    g_foc_dyn_speed_phase_u16 = FOC_Log_ToU16(phase, 65535.0f / FOC_2PI);
    g_foc_dyn_speed_ref_rpm = FOC_Log_ToI16(target, 1.0f);

    return target;
}

static void FOC_DynSpeed_WriteCoreRef(float rpm)
{
    if (rpm > s_config.motor.max_speed_rpm) {
        rpm = s_config.motor.max_speed_rpm;
    } else if (rpm < -s_config.motor.max_speed_rpm) {
        rpm = -s_config.motor.max_speed_rpm;
    }

    FOC_HAL_EnterCritical();
    s_ctx.speed_ref = rpm;
    if (rpm >= 0.0f) {
        s_ctx.direction = FOC_DIR_CW;
    } else {
        s_ctx.direction = FOC_DIR_CCW;
        s_ctx.speed_ref = -rpm;
    }
    FOC_HAL_ExitCritical();
}


static void FOC_DynSpeed_ServiceRef(void)
{
    uint32_t now_us = FOC_HAL_GetTimestampUs();

    g_foc_dyn_core_loop_count++;
    g_foc_dyn_core_state = (uint8_t)s_ctx.state;
    g_foc_dyn_core_seen_ref_rpm = FOC_Log_ToI16(s_ctx.speed_ref, 1.0f);
    g_foc_dyn_core_seen_fdb_rpm = FOC_Log_ToI16(s_ctx.speed_fdb, 1.0f);

    if (g_foc_dyn_speed_reset_stats != 0U) {
        g_foc_dyn_speed_reset_stats = 0U;
        FOC_DynSpeed_ResetStats(now_us);
    }

    if (g_foc_dyn_speed_enable == 0U) {
        s_dyn_speed_prev_enable = 0U;
        return;
    }

    if (s_dyn_speed_prev_enable == 0U) {
        s_dyn_speed_prev_enable = 1U;
        FOC_DynSpeed_ResetStats(now_us);
    }

    {
        float target = FOC_DynSpeed_CalcRef(now_us);

        if (g_foc_dyn_speed_reverse != 0U) {
            target = -target;
            g_foc_dyn_speed_ref_rpm = FOC_Log_ToI16(target, 1.0f);
        }

        FOC_DynSpeed_WriteCoreRef(target);
    }
}

static void FOC_BidirSpeed_ServiceRef(void)
{
    uint32_t now_us = FOC_HAL_GetTimestampUs();
    uint32_t period_ms;
    uint32_t period_us;
    uint32_t elapsed_us;
    uint32_t phase_us;
    float phase;
    float target;

    if (g_foc_bidir_speed_reset_stats != 0U) {
        g_foc_bidir_speed_reset_stats = 0U;
        s_bidir_speed_prev_enable = 0U;
    }

    if (g_foc_bidir_speed_enable == 0U) {
        s_bidir_speed_prev_enable = 0U;
        return;
    }

    if (s_bidir_speed_prev_enable == 0U) {
        s_bidir_speed_prev_enable = 1U;
        s_bidir_speed_start_us = now_us;
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
    target = (float)g_foc_bidir_speed_max_rpm * FOC_FastSin(phase);

    if (target > s_config.motor.max_speed_rpm) {
        target = s_config.motor.max_speed_rpm;
    } else if (target < -s_config.motor.max_speed_rpm) {
        target = -s_config.motor.max_speed_rpm;
    }

    g_foc_bidir_speed_elapsed_ms = elapsed_us / 1000U;
    g_foc_bidir_speed_phase_u16 = FOC_Log_ToU16(phase, 65535.0f / FOC_2PI);
    g_foc_bidir_speed_ref_rpm = FOC_Log_ToI16(target, 1.0f);

    g_foc_dyn_speed_elapsed_ms = g_foc_bidir_speed_elapsed_ms;
    g_foc_dyn_speed_phase_u16 = g_foc_bidir_speed_phase_u16;
    g_foc_dyn_speed_ref_rpm = g_foc_bidir_speed_ref_rpm;
    g_foc_dyn_speed_last_ext_ref_rpm = g_foc_bidir_speed_ref_rpm;

    FOC_DynSpeed_WriteCoreRef(target);
}

static void FOC_DynSpeed_RecordLog(uint32_t now_us, int16_t err_rpm)
{
    uint16_t idx;
    uint32_t decim_us = (uint32_t)g_foc_dyn_log_decim_ms * 1000U;
    float signed_speed_fdb = s_ctx.speed_fdb;
    float signed_speed_ctrl_fdb = s_ctx.speed_ctrl_fdb;

    if (s_ctx.direction == FOC_DIR_CCW) {
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
    if (s_ctx.direction == FOC_DIR_CCW) {
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

     if (s_ctx.direction == FOC_DIR_CCW) {
         offset_rad += (float)g_foc_ccw_angle_offset_mrad * 0.001f;
     }

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

static void FOC_UpdateSpeedControlFeedback(uint32_t dt_us)
{
    float alpha = FOC_SPEED_CTRL_FILTER_ALPHA;
    float prev_fdb = s_ctx.speed_ctrl_fdb;
    float next_fdb = prev_fdb;

    if ((FOC_FABS(s_speed_ref_ctrl) < 1.0f) &&
        (FOC_FABS(s_ctx.speed_fdb) < 1.0f)) {
        next_fdb = 0.0f;
    } else if ((FOC_FABS(prev_fdb) < 1.0f) &&
               (FOC_FABS(s_ctx.speed_fdb) >= 1.0f)) {
        next_fdb = s_ctx.speed_fdb;
    } else if (s_ctx.speed_fdb > s_speed_ref_ctrl) {
        next_fdb = s_ctx.speed_fdb;
    } else if (alpha >= 1.0f) {
        next_fdb = s_ctx.speed_fdb;
    } else if (alpha > 0.0f) {
        next_fdb = alpha * s_ctx.speed_fdb + (1.0f - alpha) * prev_fdb;
    }


    if ((next_fdb < prev_fdb) &&
        !((FOC_FABS(s_speed_ref_ctrl) < 1.0f) &&
          (FOC_FABS(s_ctx.speed_fdb) < 1.0f))) {
        float max_fall;

        if (dt_us == 0U) {
            dt_us = FOC_CONTROL_PERIOD_US;
        } else if (dt_us > FOC_CONTROL_PID_DT_MAX_US) {
            dt_us = FOC_CONTROL_PID_DT_MAX_US;
        }

        max_fall = FOC_SPEED_CTRL_FDB_FALL_RPM_PER_S *
                   ((float)dt_us * 1.0e-6f);
        if ((prev_fdb - next_fdb) > max_fall) {
            next_fdb = prev_fdb - max_fall;
        }
    }


    s_ctx.speed_ctrl_fdb = next_fdb;
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
     g_foc_hall_dir_reject_count = 0U;
     g_foc_hall_dir_reject_prev_sector = 0U;
     g_foc_hall_dir_reject_cur_sector = 0U;
     g_foc_hall_dir_reject_direction = 0U;
     g_foc_hall_event_used_count = 0U;
     g_foc_hall_event_seq = 0U;
     g_foc_hall_event_age_us = 0U;
     g_foc_hall_poll_count = 0U;
     g_foc_speed_ctrl_fdb_rpm = 0;
     g_foc_speed_error_boost_mA = 0;
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

 static uint8_t FOC_HallStepMatchesControlDirection(uint8_t from, uint8_t to)
 {
     uint8_t next;
     uint8_t prev;

     if ((s_speed_ref_ctrl_direction != s_ctx.direction) ||
         (s_speed_ref_ctrl < FOC_HALL_DIR_CHECK_MIN_REF_RPM) ||
         (from < 1U) || (from > 6U) || (to < 1U) || (to > 6U) ||
         (from == to)) {
         return 1U;
     }

     next = (from == 6U) ? 1U : (uint8_t)(from + 1U);
     prev = (from == 1U) ? 6U : (uint8_t)(from - 1U);

     if (s_ctx.direction == FOC_DIR_CCW) {
         return (to == prev) ? 1U : 0U;
     }

     return (to == next) ? 1U : 0U;
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
         }
         s_hall_illegal_transition_count = 0U;
         return 1U;
     }

     if (allow_missed_transition != 0U) {
         s_ctx.hall_sector = *candidate;
         s_ctx.hall_sector_timestamp_us = sector_timestamp_us;
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
         s_ctx.hall_sector = *candidate;
         s_ctx.hall_sector_timestamp_us = sector_timestamp_us;
         s_ctx.theta_e_predicted = candidate->theta_e;
         s_hall_illegal_transition_count = 0U;
         g_foc_hall_recovery_accept_count++;
         g_foc_hall_recovery_accept_prev_sector = prev_sector;
         g_foc_hall_recovery_accept_cur_sector = cur_sector;
         return 1U;
     }

     if (FOC_HallStepMatchesControlDirection(prev_sector, cur_sector) == 0U) {
         g_foc_hall_dir_reject_count++;
         g_foc_hall_dir_reject_prev_sector = prev_sector;
         g_foc_hall_dir_reject_cur_sector = cur_sector;
         g_foc_hall_dir_reject_direction = (uint8_t)s_ctx.direction;
         return 0U;
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

     s_ctx.hall_sector = *candidate;
     s_ctx.hall_sector_timestamp_us = sector_timestamp_us;
     s_hall_illegal_transition_count = 0U;
     return 1U;
 }

 

 /* ===================================================================

  *  初始化 / 反初始化

  * =================================================================== */

 

 int FOC_Core_Init(const FOC_Config_t *config)

 {

     if (config == NULL) {

         return FOC_ERR;

     }

 

     /* 初始化驱动层 */

     if (FOC_HAL_Init() != 0) {

         return FOC_ERR;

     }

 

     /* 初始化 sin/cos 查找表 */

     FOC_Math_InitTable();

 

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

 

     /* 默认方向正转 */

     s_ctx.direction = FOC_DIR_CW;

 

     /* 进入待机态 */

     s_ctx.state = FOC_STATE_IDLE;

     s_ctx.fault = FOC_FAULT_NONE;

 

     return FOC_OK;

 }

 

 int FOC_Core_DeInit(void)

 {

     /* 确保已停止 */

     if (s_ctx.state == FOC_STATE_RUNNING) {

         FOC_Core_Stop();

     }

 

     FOC_HAL_DisablePWM();

     memset(&s_ctx, 0, sizeof(FOC_Context_t));

     s_ctx.state = FOC_STATE_INIT;

 

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
     s_speed_error_boost_prev_ref = 0.0f;



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
     g_foc_dyn_speed_enable = 0U;
     g_foc_bidir_speed_enable = 0U;
     s_dyn_speed_prev_enable = 0U;
     s_bidir_speed_prev_enable = 0U;
     g_foc_dyn_speed_reset_stats = 0U;
     g_foc_bidir_speed_reset_stats = 0U;

     s_ctx.iq_ref    = 0.0f;
     s_ctx.speed_ctrl_fdb = 0.0f;
     g_foc_speed_ctrl_fdb_rpm = 0;
     s_ctx.speed_loop_counter = 0U;
     s_speed_loop_accum_us = 0U;
     s_speed_error_boost_prev_ref = 0.0f;



     /* 进入待机态 */

     s_ctx.state = FOC_STATE_IDLE;

 

     return FOC_OK;

 }

 

 /* ===================================================================

  *  主控制循环

  * =================================================================== */

 

 void FOC_Core_MainLoop(void)

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

     if (g_foc_bidir_speed_enable != 0U) {
         FOC_BidirSpeed_ServiceRef();
     } else {
         FOC_DynSpeed_ServiceRef();
     }

     FOC_NormalizeSignedSpeedRef();

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

     float theta_e_ctrl = FOC_Observer_PredictAngle(&s_ctx, observer_dt,

                                                     s_config.motor.pole_pairs);

 

     s_ctx.speed_fdb = FOC_Observer_CalcSpeed(&s_ctx, s_ctx.theta_e,

                                               observer_dt, s_config.motor.pole_pairs);
     FOC_UpdateSpeedRefRamp(control_period_us);
     FOC_UpdateSpeedControlFeedback(control_period_us);

     if (s_ctx.sector_no_change_count >= FOC_SECTOR_NO_CHANGE_THRESHOLD) {
         theta_e_ctrl = s_ctx.theta_e_predicted;
     }

 


 

     theta_e_ctrl = FOC_ApplyCurrentAngleTrim(theta_e_ctrl);

 

     /* ---- 3. Clarke 变换 ---- */

     FOC_Clarke(&s_ctx.i_abc, &s_ctx.i_ab);

 

     /* ---- 4. Park 变换 ---- */

     FOC_Park(&s_ctx.i_ab, theta_e_ctrl, &s_ctx.i_dq);
     FOC_UpdateCurrentAngleTrim(pid_dt);

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

         if (s_speed_loop_accum_us >= speed_loop_period_us) {
             float speed_dt = (float)s_speed_loop_accum_us * 1.0e-6f;
             float speed_inv_dt = 1000000.0f / (float)s_speed_loop_accum_us;
             float speed_ref_ctrl = s_speed_ref_ctrl;
             float speed_error = speed_ref_ctrl - s_ctx.speed_ctrl_fdb;
             float speed_iq_ref;

             s_speed_loop_accum_us = 0U;
             s_ctx.speed_loop_counter = 0U;

             speed_iq_ref = FOC_PID_Update(&s_ctx.pid_speed,
                                            speed_error,
                                            speed_dt, speed_inv_dt);

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
             s_speed_error_boost_prev_ref = speed_ref_ctrl;

             speed_iq_ref = FOC_LimitRegenBrakingIq(speed_iq_ref);
             s_ctx.iq_ref = FOC_CLAMP(speed_iq_ref,
                                      s_ctx.pid_speed.out_min,
                                      s_ctx.pid_speed.out_max);

         }
     }

 

     /* ---- 7. 电流环 PID ---- */

     s_ctx.v_dq.d = FOC_PID_Update(&s_ctx.pid_id,

                                     s_ctx.id_ref - s_ctx.i_dq.d,

                                     pid_dt, pid_inv_dt);

     s_ctx.v_dq.q = FOC_PID_Update(&s_ctx.pid_iq,

                                     FOC_ControlIqRef() - s_ctx.i_dq.q,

                                     pid_dt, pid_inv_dt);

 

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

 

     FOC_Log_Record(theta_e_ctrl);
     FOC_DynSpeed_ServiceMetrics();
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

 

 const FOC_Context_t *FOC_Core_GetContext(void)

 {

     return &s_ctx;

 }

 

 int FOC_Core_SetSpeedRef(float rpm)

 {

     /* 限幅到电机最大转速 */


     if (rpm > s_config.motor.max_speed_rpm) {

         rpm = s_config.motor.max_speed_rpm;

     } else if (rpm < -s_config.motor.max_speed_rpm) {

         rpm = -s_config.motor.max_speed_rpm;

     }

 

     /* 临界区保护：speed_ref 被 ISR 中的主循环读取 */

     FOC_HAL_EnterCritical();

     s_ctx.speed_ref = rpm;

 

     /* 根据转速正负自动判断方向 */

     if (rpm >= 0.0f) {

         s_ctx.direction = FOC_DIR_CW;

     } else {

         s_ctx.direction = FOC_DIR_CCW;

         s_ctx.speed_ref = -rpm; /* 速度绝对值用于PID，方向由电角度处理 */

     }

     FOC_HAL_ExitCritical();

 

     return FOC_OK;

 }

 

 int FOC_Core_SetDirection(FOC_Dir_e dir)

 {

     FOC_HAL_EnterCritical();

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
     s_hall_illegal_transition_count = 0U;

     s_ctx.state = FOC_STATE_IDLE;

 

     return FOC_OK;

 }

 


