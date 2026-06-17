
/**

 * @file foc_config.h

 * @brief FOC 电机控制模块 — 编译期配置参数

 *

 * 本文件定义 FOC 模块的编译期默认值与可配宏。

 * 运行时参数通过 FOC_Config_t 传入，此处仅提供默认值与硬件约束宏。

 */

 

#ifndef FOC_CONFIG_H

#define FOC_CONFIG_H

 

#ifdef __cplusplus

extern "C" {

#endif

/* ===================================================================

 *  数学常量

 * =================================================================== */

 

#ifndef FOC_PI

#define FOC_PI              3.14159265358979323846f

#endif

 

#ifndef FOC_2PI

#define FOC_2PI             6.28318530717958647692f

#endif

 

#ifndef FOC_SQRT3

#define FOC_SQRT3           1.73205080756887729352f

#endif

 

#ifndef FOC_SQRT3_DIV2

#define FOC_SQRT3_DIV2      0.86602540378443864676f

#endif

 

#ifndef FOC_1_DIV_SQRT3

#define FOC_1_DIV_SQRT3     0.57735026918962576451f

#endif

 

#ifndef FOC_2_DIV_SQRT3

#define FOC_2_DIV_SQRT3     1.15470053837925152902f

#endif

 

/* ===================================================================

 *  控制环路默认参数

 * =================================================================== */

 

/** 默认控制环路频率 (Hz)，典型值 10kHz */

#ifndef FOC_CONTROL_FREQ_HZ

#define FOC_CONTROL_FREQ_HZ     10000U

#endif

 

/** 默认控制周期 (s) */

#define FOC_CONTROL_PERIOD_S     (1.0f / (float)FOC_CONTROL_FREQ_HZ)

#ifndef FOC_CONTROL_PERIOD_US

#define FOC_CONTROL_PERIOD_US    (1000000U / FOC_CONTROL_FREQ_HZ)

#endif

#ifndef FOC_CONTROL_LATE_PERIOD_US

#define FOC_CONTROL_LATE_PERIOD_US      ((FOC_CONTROL_PERIOD_US * 3U) / 2U)

#endif

#ifndef FOC_CONTROL_OBSERVER_DT_MAX_US

#define FOC_CONTROL_OBSERVER_DT_MAX_US  300U

#endif

#ifndef FOC_CONTROL_RECOVERY_PERIOD_US

#define FOC_CONTROL_RECOVERY_PERIOD_US  FOC_CONTROL_OBSERVER_DT_MAX_US

#endif

#ifndef FOC_CONTROL_RECOVERY_ENABLE

#define FOC_CONTROL_RECOVERY_ENABLE  0

#endif

#ifndef FOC_RECOVERY_ZERO_VECTOR_MIN_CYCLES

#define FOC_RECOVERY_ZERO_VECTOR_MIN_CYCLES  5U

#endif

#ifndef FOC_RECOVERY_ZERO_VECTOR_MAX_CYCLES

#define FOC_RECOVERY_ZERO_VECTOR_MAX_CYCLES  30U

#endif

#ifndef FOC_RECOVERY_RELEASE_CURRENT_A

#define FOC_RECOVERY_RELEASE_CURRENT_A  2.0f

#endif

#ifndef FOC_HALL_RECOVERY_ACCEPT_CYCLES

#define FOC_HALL_RECOVERY_ACCEPT_CYCLES  10U

#endif

#ifndef FOC_POST_RECOVERY_DUTY_SLEW_CYCLES

#define FOC_POST_RECOVERY_DUTY_SLEW_CYCLES  3U

#endif

#ifndef FOC_POST_RECOVERY_DUTY_STEP_MAX

#define FOC_POST_RECOVERY_DUTY_STEP_MAX  0.15f

#endif

#ifndef FOC_CONTROL_PID_DT_MAX_US

#define FOC_CONTROL_PID_DT_MAX_US       300U

#endif

 

/* ===================================================================

 *  保护阈值默认值

 * =================================================================== */

 

/** 过流保护阈值 (A)，超过此值触发 FOC_FAULT_OVERCURRENT */

#ifndef FOC_OVERCURRENT_THRESHOLD_A

#define FOC_OVERCURRENT_THRESHOLD_A     10.0f

#endif

 

/** 过压保护阈值 (V)，母线电压超过此值触发 FOC_FAULT_OVERVOLTAGE */

#ifndef FOC_OVERVOLTAGE_THRESHOLD_V

#define FOC_OVERVOLTAGE_THRESHOLD_V     15.0f

#endif

 

/** 欠压保护阈值 (V)，母线电压低于此值触发 FOC_FAULT_UNDERVOLTAGE */

#ifndef FOC_UNDERVOLTAGE_THRESHOLD_V

#define FOC_UNDERVOLTAGE_THRESHOLD_V    9.0f

#endif

 

/** 堵转判定：连续N个控制周期速度误差超过阈值则判定堵转 */

#ifndef FOC_STALL_COUNT_THRESHOLD

#define FOC_STALL_COUNT_THRESHOLD       2000U

#endif

 

/** 堵转判定速度误差阈值 (rpm) */

#ifndef FOC_STALL_SPEED_ERROR_RPM

#define FOC_STALL_SPEED_ERROR_RPM       200.0f

#endif

 

/** 霍尔传感器异常检测：无效扇区持续计数阈值 */

/* Start-fail/stall detection: valid command but feedback remains near zero. */
#ifndef FOC_STALL_MIN_SPEED_REF_RPM

#define FOC_STALL_MIN_SPEED_REF_RPM     300.0f

#endif

#ifndef FOC_STALL_MAX_FEEDBACK_RPM

#define FOC_STALL_MAX_FEEDBACK_RPM      100.0f

#endif

/* Hall invalid-sector persistence threshold. */
#ifndef FOC_HALL_INVALID_COUNT_THRESHOLD

#define FOC_HALL_INVALID_COUNT_THRESHOLD   1000U

#endif

 

/** 连续无扇区跳变的控制周期数阈值，超过后认为电机已停止 */

#ifndef FOC_SECTOR_NO_CHANGE_THRESHOLD

#define FOC_SECTOR_NO_CHANGE_THRESHOLD  500U

#endif

#ifndef FOC_HALL_STOP_TIMEOUT_RATIO
#define FOC_HALL_STOP_TIMEOUT_RATIO  3U
#endif

#ifndef FOC_HALL_STOP_TIMEOUT_MIN_US
#define FOC_HALL_STOP_TIMEOUT_MIN_US  1000U
#endif

 

/** 速度环降采样比：电流环每执行 N 次，速度环执行 1 次 */

#ifndef FOC_SPEED_LOOP_DOWNSAMPLE

#define FOC_SPEED_LOOP_DOWNSAMPLE       10U

#endif

 

/** 角度预测软同步系数 (0~1)：扇区跳变时预测角度向扇区中心角度靠拢的比例，

 * 1.0=硬同步（立即跳到扇区中心），0.0=不同步。0.2~0.4可避免角度突变 */

#ifndef FOC_ANGLE_SYNC_FACTOR

#define FOC_ANGLE_SYNC_FACTOR           0.1f

#endif

#ifndef FOC_ANGLE_SYNC_STEP_MAX_RAD

#define FOC_ANGLE_SYNC_STEP_MAX_RAD     (FOC_PI / 36.0f)

#endif

#ifndef FOC_HALL_EDGE_SYNC_ENABLE

#define FOC_HALL_EDGE_SYNC_ENABLE       0

#endif

#ifndef FOC_HALL_EDGE_SYNC_FACTOR

#define FOC_HALL_EDGE_SYNC_FACTOR       1.0f

#endif

#ifndef FOC_HALL_EDGE_SYNC_STEP_MAX_RAD

#define FOC_HALL_EDGE_SYNC_STEP_MAX_RAD (FOC_PI / 12.0f)

#endif

#ifndef FOC_HALL_EDGE_SYNC_DT_FRACTION

#define FOC_HALL_EDGE_SYNC_DT_FRACTION  0.5f

#endif

#ifndef FOC_HALL_EDGE_SYNC_ADVANCE_MAX_RAD

#define FOC_HALL_EDGE_SYNC_ADVANCE_MAX_RAD  (FOC_PI / 12.0f)

#endif

#ifndef FOC_ANGLE_SYNC_RECOVERY_SPEED_ERROR_RPM

#define FOC_ANGLE_SYNC_RECOVERY_SPEED_ERROR_RPM  150.0f

#endif

#ifndef FOC_ANGLE_SYNC_RECOVERY_DIFF_RAD

#define FOC_ANGLE_SYNC_RECOVERY_DIFF_RAD  (FOC_PI / 4.0f)

#endif

#ifndef FOC_ANGLE_SYNC_RECOVERY_FACTOR

#define FOC_ANGLE_SYNC_RECOVERY_FACTOR   0.2f

#endif

#ifndef FOC_ANGLE_SYNC_RECOVERY_STEP_MAX_RAD

#define FOC_ANGLE_SYNC_RECOVERY_STEP_MAX_RAD  (FOC_PI / 18.0f)

#endif

#ifndef FOC_ANGLE_SYNC_RESYNC_DIFF_RAD

#define FOC_ANGLE_SYNC_RESYNC_DIFF_RAD   (FOC_PI / 3.0f)

#endif

#ifndef FOC_LATE_PERIOD_ANGLE_SYNC_FACTOR

#define FOC_LATE_PERIOD_ANGLE_SYNC_FACTOR  FOC_ANGLE_SYNC_FACTOR

#endif

#ifndef FOC_LATE_PERIOD_ANGLE_SYNC_STEP_MAX_RAD

#define FOC_LATE_PERIOD_ANGLE_SYNC_STEP_MAX_RAD  FOC_ANGLE_SYNC_STEP_MAX_RAD

#endif

#ifndef FOC_CURRENT_ANGLE_TRIM_ENABLE

#define FOC_CURRENT_ANGLE_TRIM_ENABLE   0

#endif

#ifndef FOC_CURRENT_ANGLE_TRIM_GAIN

#define FOC_CURRENT_ANGLE_TRIM_GAIN     1.0f

#endif

#ifndef FOC_CURRENT_ANGLE_TRIM_MAX_RAD

#define FOC_CURRENT_ANGLE_TRIM_MAX_RAD  (FOC_PI / 9.0f)

#endif

#ifndef FOC_CURRENT_ANGLE_TRIM_MIN_IQ_A

#define FOC_CURRENT_ANGLE_TRIM_MIN_IQ_A 0.3f

#endif

#ifndef FOC_CURRENT_ANGLE_TRIM_MIN_SPEED_RPM

#define FOC_CURRENT_ANGLE_TRIM_MIN_SPEED_RPM  500.0f

#endif

#ifndef FOC_CURRENT_ANGLE_TRIM_MAX_SPEED_ERR_RPM

#define FOC_CURRENT_ANGLE_TRIM_MAX_SPEED_ERR_RPM  150.0f

#endif

#ifndef FOC_CURRENT_ANGLE_TRIM_DECAY

#define FOC_CURRENT_ANGLE_TRIM_DECAY    0.995f

#endif

#ifndef FOC_ANGLE_LOSS_RECOVERY_ENABLE

#define FOC_ANGLE_LOSS_RECOVERY_ENABLE  1

#endif

#ifndef FOC_ANGLE_LOSS_RECOVERY_CURRENT_A

#define FOC_ANGLE_LOSS_RECOVERY_CURRENT_A  4.0f

#endif

#ifndef FOC_ANGLE_LOSS_RECOVERY_ID_A

#define FOC_ANGLE_LOSS_RECOVERY_ID_A    3.0f

#endif

#ifndef FOC_ANGLE_LOSS_RECOVERY_ID_IQ_RATIO

#define FOC_ANGLE_LOSS_RECOVERY_ID_IQ_RATIO  1.5f

#endif

#ifndef FOC_STARTUP_PREDICT_START_RPM

#define FOC_STARTUP_PREDICT_START_RPM   200.0f

#endif

#ifndef FOC_STARTUP_PREDICT_MAX_RPM

#define FOC_STARTUP_PREDICT_MAX_RPM     600.0f

#endif

#ifndef FOC_STARTUP_PREDICT_RAMP_RPM_PER_S

#define FOC_STARTUP_PREDICT_RAMP_RPM_PER_S  3000.0f

#endif

/* Hall electrical angle calibration.
 * The Hall lookup table stores sector center angles for compatibility.
 * Control-angle synchronization uses the sector entry edge when
 * FOC_HALL_EDGE_SYNC_ENABLE is set.
 */
#ifndef FOC_HALL_ANGLE_OFFSET_RAD
#define FOC_HALL_ANGLE_OFFSET_RAD       0.0f
#endif

/* Reject Hall sector changes faster than this fraction of the theoretical
 * sector time at FOC_SPEED_ESTIMATE_MAX_RPM.
 */
#ifndef FOC_HALL_MIN_SECTOR_TIME_RATIO
#define FOC_HALL_MIN_SECTOR_TIME_RATIO  0.40f
#endif

/* Consecutive illegal Hall transitions required before entering fault state.
 * Single-sample non-adjacent jumps are treated as glitches and ignored.
 */
#ifndef FOC_HALL_ILLEGAL_TRANSITION_FAULT_COUNT
#define FOC_HALL_ILLEGAL_TRANSITION_FAULT_COUNT  5U
#endif

 

/* ===================================================================

 *  算法配置

 * =================================================================== */

 

/** 速度观测低通滤波系数 (0~1)，值越大跟踪越快但噪声越大 */

#ifndef FOC_SPEED_FILTER_ALPHA

#define FOC_SPEED_FILTER_ALPHA          0.05f

#endif

 

/** Speed estimate clamp (rpm), used to reject abnormal Hall jump spikes. */

#ifndef FOC_SPEED_ESTIMATE_MAX_RPM

#define FOC_SPEED_ESTIMATE_MAX_RPM      4000.0f

#endif

 

/** SVPWM 调制系数上限（留余量，避免过调制） */

#ifndef FOC_SVPWM_MODULATION_MAX

#define FOC_SVPWM_MODULATION_MAX        0.95f

#endif

 

/** PID 积分抗饱和：输出限幅时停止积分累积 */

#ifndef FOC_PID_ANTI_WINDUP

#define FOC_PID_ANTI_WINDUP             1

#endif

 

/* ===================================================================

 *  返回值定义

 * =================================================================== */

 

#define FOC_OK      0   /**< 操作成功 */

#define FOC_ERR    (-1)  /**< 通用错误 */

#define FOC_BUSY   (-2)  /**< 模块忙（状态不允许该操作） */

#define FOC_FAULT  (-3)  /**< 模块处于故障态 */

 

#ifdef __cplusplus

}

#endif

 

#endif /* FOC_CONFIG_H */

 

 


