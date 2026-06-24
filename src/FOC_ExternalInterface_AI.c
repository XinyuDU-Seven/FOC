
#include "FOC_ExternalInterface.h"

#include <string.h>

#include "foc_api.h"
#include "foc_core.h"
#include "foc_hal_if.h"
#include "foc_math.h"

#ifdef __ICCARM__
#define FOC_AI_DEBUG_ROOT __root
#else
#define FOC_AI_DEBUG_ROOT
#endif

FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_count = 0U;
extern volatile uint8_t g_foc_dyn_speed_start_on_max_fdb;
extern volatile float speed_ref;

#define FOC_DYN_SPEED_LOG_SIZE 128U

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

FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_speed_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_speed_step_enable = 0U;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_speed_reset_stats = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_speed_period_ms = 8000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_speed_max_rpm = 4000U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_speed_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_speed_phase_u16 = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_bidir_speed_ref_rpm = 0;

/* LiveWatch: 0 stop, 1 fixed, 2 +1000..+4000 sine,
 * 3 -2000..+2000 sine, 4 -1000..-4000 sine.
 */
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_select = FOC_TEST_CASE_STOP;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_applied = FOC_TEST_CASE_STOP;
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_test_case_last_error = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_test_case_exec_count = 0U;

static uint8_t s_foc_test_case_last_select = FOC_TEST_CASE_STOP;

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

#define FOC_APP_MOTOR_ID       0U
#define FOC_APP_POLE_PAIRS     4U
#define FOC_APP_DIR_NONE       0U
#define FOC_APP_DIR_FORWARD    1U
#define FOC_APP_DIR_REVERSE    2U
#define FOC_APP_MODE_SPEED     1U
#define FOC_APP_MODE_CURRENT   3U

static FocError FOC_AI_CheckMotorId(uint8_t unId)
{
  return (unId == FOC_APP_MOTOR_ID) ? FOC_SUCCESS : FOC_MOTOR_ID_INVALID;
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
  return FOC_INPUT_PARAMETER_INVALID;
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
  FocError err = FOC_AI_CheckMotorId(unId);

  if (err != FOC_SUCCESS) {
    return err;
  }

  ctx = FOC_Core_GetContext();
  if (ctx->state == FOC_STATE_RUNNING) {
    return FOC_SUCCESS;
  }
  if (ctx->state == FOC_STATE_FAULT) {
    return FOC_AI_MapFault(ctx->fault);
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
  FocError err = FOC_AI_CheckMotorId(unId);

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
  FocError err = FOC_AI_CheckMotorId(unId);

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
  float target;
  FocError err;

  (void)unParam2;
  (void)unParam3;

  err = FOC_AI_CheckMotorId(unId);
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
  FocError err = FOC_AI_CheckMotorId(unId);

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
  const FOC_Context_t *ctx;
  FocError err;

  if (pstMotorFullStates == NULL) {
    return FOC_POINTER_NULL;
  }

  err = FOC_AI_CheckMotorId(unId);
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
  (void)unMotorID;

  if (punMotorNum == NULL) {
    return FOC_POINTER_NULL;
  }

  *punMotorNum = FOC_APP_MOTOR_ID;
  return FOC_SUCCESS;
}

 

float gfSpeedTarget = 0;

uint8_t gunCtrl = 0;

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

static void FOC_TestCase_Apply(uint8_t test_case)
{
  uint8_t unId = 0U;

  g_foc_test_case_last_error = 0U;
  speed_ref = -1.0f;

  if(test_case == FOC_TEST_CASE_STOP){

    FOC_TestCase_ClearAutoModes();

    Foc_DisableFocControl(unId);

  }else if(test_case == FOC_TEST_CASE_FIXED_SPEED){

    const FOC_Context_t *ctx = FOC_Core_GetContext();
    float fixed_ref = ctx->speed_ref;

    if ((fixed_ref > 0.0f) && (ctx->direction == FOC_DIR_CCW)) {
      fixed_ref = -fixed_ref;
    }

    FOC_TestCase_ClearAutoModes();
    g_foc_dyn_speed_start_on_max_ref = 0U;
    g_foc_dyn_speed_start_on_max_fdb = 0U;

    Foc_EnableFocControl(unId);

    Foc_SetSpeedReference(unId, fixed_ref);

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

    g_foc_bidir_speed_max_rpm = 4000U;
    g_foc_bidir_speed_step_enable = 0U;
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
  g_foc_test_case_exec_count++;
}
static void FOC_TestCase_Service(void)
{
  uint8_t test_case = g_foc_test_case_select;

  if(test_case == s_foc_test_case_last_select){
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
