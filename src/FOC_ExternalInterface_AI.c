
#include "FOC_ExternalInterface.h"

#include "foc_api.h"
#include "foc_core.h"
#include "foc_hal_if.h"
#include "foc_math.h"
#include "foc_config.h"

#ifdef __ICCARM__
#define FOC_AI_DEBUG_ROOT __root
#else
#define FOC_AI_DEBUG_ROOT
#endif

FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_period_us = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_max_period_us = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_max_period_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_late_count = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_late_period_us = 0U;
static uint32_t s_foc_ai_callback_last_us = 0U;

#define FOC_DYN_SPEED_LOG_SIZE 128U

#define FOC_TEST_CASE_STOP             0U
#define FOC_TEST_CASE_FIXED_SPEED      1U
#define FOC_TEST_CASE_DYN_SPEED        2U
#define FOC_TEST_CASE_BIDIR_SPEED      3U
#define FOC_TEST_CASE_NEG_DYN_SPEED    4U

#define FOC_TEST_CASE_DYN_MIN_RPM       1000U
#define FOC_TEST_CASE_DYN_MAX_RPM       4000U
#define FOC_TEST_CASE_DYN_PERIOD_MS     4000U

#define FOC_TEST_CASE_NEG_DYN_MIN_RPM   1000U
#define FOC_TEST_CASE_NEG_DYN_MAX_RPM   4000U
#define FOC_TEST_CASE_NEG_DYN_PERIOD_MS 4000U

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
FOC_AI_DEBUG_ROOT volatile uint8_t  g_foc_bidir_speed_reset_stats = 0U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_speed_period_ms = 8000U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_speed_max_rpm = 2000U;
FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_bidir_speed_elapsed_ms = 0U;
FOC_AI_DEBUG_ROOT volatile uint16_t g_foc_bidir_speed_phase_u16 = 0U;
FOC_AI_DEBUG_ROOT volatile int16_t  g_foc_bidir_speed_ref_rpm = 0;

/* LiveWatch: change select to 0/1/2/3; applied once on value change. */
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

static void FOC_TestCase_Service(void);

/*******************************************************************************************

  函数名称:  Foc_AlgorithmControlCallback

  函数功能:  电机的Foc中断函数，基于电流环的控制目标完成三相电机占空比输出

  输入参数：  无

  输出参数： 无

  返回值:    无

 *******************************************************************************************/

void Foc_AlgorithmControlCallback_AI(void){
  uint32_t now_us = FOC_HAL_GetTimestampUs();
  uint32_t period_us;

  if (s_foc_ai_callback_last_us != 0U) {
    period_us = now_us - s_foc_ai_callback_last_us;
    g_foc_ai_callback_period_us = period_us;
    if (period_us > g_foc_ai_callback_max_period_us) {
      g_foc_ai_callback_max_period_us = period_us;
      g_foc_ai_callback_max_period_count = g_foc_ai_callback_count;
    }
    if (period_us > FOC_CONTROL_LATE_PERIOD_US) {
      g_foc_ai_callback_late_count++;
      g_foc_ai_callback_late_period_us = period_us;
    }
  }
  s_foc_ai_callback_last_us = now_us;

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

void Foc_Init_AI(void){

  FOC_Config_t config;

  memset(&config,0,sizeof(FOC_Config_t));

  config.motor.pole_pairs = 4;

  config.motor.rs = 0.65;

  config.motor.ls_d = 0.00007;

  config.motor.ls_q = 0.00007;

  config.motor.v_bus = 12;

  config.motor.max_speed_rpm = 4000;

  config.motor.max_current_a = 5;

  config.current_d_pid.kp = 0.25f;

  config.current_d_pid.ki = 60.0f;

  config.current_d_pid.kd = 0;

  config.current_d_pid.out_max = 10;

  config.current_d_pid.out_min = -10;

  config.current_q_pid.kp = 0.25f;

  config.current_q_pid.ki = 60.0f;

  config.current_q_pid.kd = 0;

  config.current_q_pid.out_max = 10;

  config.current_q_pid.out_min = -10;

  config.speed_pid.kp = 0.0065f;

  config.speed_pid.ki = 0.0008f;

  config.speed_pid.kd = 0.0f;

  config.speed_pid.out_max = config.motor.max_current_a;

  config.speed_pid.out_min = -config.motor.max_current_a;

  FOC_Init(&config);

  FOC_Start();

}

 

/*******************************************************************************************

  函数名称:  Foc_EnableFocControl

  函数功能:  激活电机的FOC电流环控制输出

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_EnableFocControl_AI(uint8_t unId){

    (void)unId;

    FOC_Start();

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_DisableFocControl

  函数功能:  停止电机的FOC电流环控制输出

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_DisableFocControl_AI(uint8_t unId){

    (void)unId;

    FOC_Stop();

    return 0;

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

FocError Foc_SetCurrentReference_AI(uint8_t unId, float fId, float fIq){

    (void)unId;

    (void)fId;

    (void)fIq;

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_SetHybridControlReference

  函数功能:  无刷电机混合控制接口

   *******************************************************************************************/

FocError Foc_SetHybridControlReference_AI(uint8_t unId, uint8_t unMode, uint16_t unParam1, uint16_t unParam2,

                                       uint16_t unParam3, uint16_t unParam4, uint16_t unParam5)

{

    (void)unId;

    (void)unMode;

    (void)unParam1;

    (void)unParam2;

    (void)unParam3;

    (void)unParam4;

    (void)unParam5;

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_SetSpeedReference

  函数功能:  设置速度闭环运行电机

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

            fSpeed：电机目标转速，RPM单位

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_SetSpeedReference_AI(uint8_t unId, float fSpeed){

    (void)unId;

    g_foc_bidir_speed_enable = 0U;

    FOC_SetSpeedRef(fSpeed);

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_GetMotorFullParameters

  函数功能:  获取电机当前状态

  输入参数: unId：设备号，0：水平无刷电机  1：坐盆无刷电机

  输出参数: pstMotorFullStates：当前电机状态的所有参数的结构体指针

  返回值:  FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_GetMotorFullParameters_AI(uint8_t unId, MotorFullStates *pstMotorFullStates){

    (void)unId;

    (void)pstMotorFullStates;

    return 0;

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

FocError Foc_GetMotorNum_AI(uint8_t unCarConfigID, uint8_t unSeatID, uint8_t unMotorID, uint8_t *punMotorNum){

    (void)unCarConfigID;

    (void)unSeatID;

    return 0;

}

 

float gfSpeedTarget = 0;

uint8_t gunCtrl = 0;

static void FOC_TestCase_ClearAutoModes(void)
{
  g_foc_dyn_speed_enable = 0U;
  g_foc_dyn_speed_reverse = 0U;
  g_foc_dyn_speed_reset_stats = 0U;
  g_foc_bidir_speed_enable = 0U;
  g_foc_bidir_speed_reset_stats = 0U;
}

static void FOC_TestCase_Apply(uint8_t test_case)
{
  uint8_t unId = 0U;

  g_foc_test_case_last_error = 0U;

  if(test_case == FOC_TEST_CASE_STOP){

    FOC_TestCase_ClearAutoModes();

    Foc_DisableFocControl(unId);

  }else if(test_case == FOC_TEST_CASE_FIXED_SPEED){

    const FOC_Context_t *ctx = FOC_Core_GetContext();
    float fixed_ref = ctx->speed_ref;

    FOC_TestCase_ClearAutoModes();

    Foc_EnableFocControl(unId);

    Foc_SetSpeedReference(unId, fixed_ref);

  }else if(test_case == FOC_TEST_CASE_DYN_SPEED){

    Foc_EnableFocControl(unId);

    g_foc_bidir_speed_enable = 0U;
    g_foc_bidir_speed_reset_stats = 0U;

    g_foc_dyn_speed_period_ms = FOC_TEST_CASE_DYN_PERIOD_MS;
    g_foc_dyn_speed_min_rpm = FOC_TEST_CASE_DYN_MIN_RPM;
    g_foc_dyn_speed_max_rpm = FOC_TEST_CASE_DYN_MAX_RPM;
    g_foc_dyn_speed_reverse = 0U;
    g_foc_dyn_speed_enable = 1U;

    g_foc_dyn_speed_reset_stats = 1U;

  }else if(test_case == FOC_TEST_CASE_NEG_DYN_SPEED){

    Foc_EnableFocControl(unId);

    g_foc_bidir_speed_enable = 0U;
    g_foc_bidir_speed_reset_stats = 0U;

    g_foc_dyn_speed_period_ms = FOC_TEST_CASE_NEG_DYN_PERIOD_MS;
    g_foc_dyn_speed_min_rpm = FOC_TEST_CASE_NEG_DYN_MIN_RPM;
    g_foc_dyn_speed_max_rpm = FOC_TEST_CASE_NEG_DYN_MAX_RPM;
    g_foc_dyn_speed_reverse = 1U;
    g_foc_dyn_speed_enable = 1U;

    g_foc_dyn_speed_reset_stats = 1U;

  }else if(test_case == FOC_TEST_CASE_BIDIR_SPEED){

    Foc_EnableFocControl(unId);

    g_foc_dyn_speed_enable = 0U;
    g_foc_dyn_speed_reverse = 0U;
    g_foc_dyn_speed_reset_stats = 0U;

    g_foc_bidir_speed_enable = 1U;

    g_foc_bidir_speed_reset_stats = 1U;

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

 

 

 


