
/**

 * @file foc_observer.h

 * @brief FOC 观测器 — 霍尔传感器原始数据解析与角度/速度估算

 *

 * 职责：

 *   1. 霍尔三路数字电平 → 扇区号解析（1~6）

 *   2. 扇区号 → 电角度映射（查找表）

 *   3. M法速度估算（扇区跳变差分法）

 *   4. 三相电流 ADC 原始值 → 实际电流换算

 *   5. 母线电压 ADC 原始值 → 实际电压换算

 *

 * 本模块是驱动层原始数据与 FOC 算法之间的解析桥梁，

 * 驱动层仅返回硬件寄存器值，所有物理量换算在此完成。

 */

 

 #ifndef FOC_OBSERVER_H

 #define FOC_OBSERVER_H

 

 #include "foc_types.h"

 

 #ifdef __cplusplus

 extern "C" {

 #endif

 

 /**

  * @brief 初始化观测器（清零历史状态）

  * @param ctx FOC 运行上下文

  */

 void FOC_Observer_Init(FOC_Context_t *ctx);

 

 /**

  * @brief 将霍尔三路数字电平解析为扇区号和电角度

  *

  * 三路霍尔信号 H1/H2/H3 组合为3位编码，映射到6个扇区：

  *   H1 H2 H3 → sector

  *    1  0  1  →  1  (0° ~ 60°)

  *    1  0  0  →  2  (60° ~ 120°)

  *    1  1  0  →  3  (120° ~ 180°)

  *    0  1  0  →  4  (180° ~ 240°)

  *    0  1  1  →  5  (240° ~ 300°)

  *    0  0  1  →  6  (300° ~ 360°)

  * 其他组合为无效状态，返回 sector=0。

  *

  * 每个扇区对应 60° 电角度区间，取扇区中心角度作为估算值。

  *

  * @param hall   霍尔传感器三路原始电平

  * @param[out] result 扇区号与电角度

  */

 /* theta_e uses the calibrated Hall sync angle, defaulting to sector entry edge. */
 void FOC_Observer_HallRawToSector(const FOC_HallRaw_t *hall,

                                    FOC_HallSector_t *result);

 

 /**

  * @brief 根据扇区号估算电角度

  *

  * 将扇区号映射到该扇区中心对应的电角度（弧度）。

  * 扇区 1~6 分别对应 30°, 90°, 150°, 210°, 270°, 330° 电角度。

  *

  * @param sector 扇区号 (1~6)，0表示无效

  * @return       电角度 (rad)，范围 [0, 2π)，无效时返回上一次角度

  */

/* Returns the calibrated Hall sync angle, not the raw sector center angle. */
float FOC_Observer_HallSectorToElecAngle(uint8_t sector);

/* Returns the calibrated Hall edge angle used by observer edge sync. */
float FOC_Observer_HallEdgeSyncAngle(const FOC_Context_t *ctx,
                                     uint8_t sector,
                                     float omega_e,
                                     float advance_max);

/* Runtime Hall angle offsets, in mrad. Indexed by physical motor id. */
extern volatile int16_t g_foc_hall_angle_offset_mrad_motor[];
int16_t FOC_Observer_GetHallAngleOffsetMrad(void);
void FOC_Observer_SetHallAngleOffsetMrad(uint8_t motor_id,
                                         int16_t offset_mrad);

 

/**

  * @brief 估算电机转速

  *

  * 采用 M 法（扇区跳变差分法）：

  *   每次扇区跳变，角度变化 60°（π/3 rad）电角度，

  *   使用两次跳变间的实际时间间隔（硬件时间戳）计算转速，

  *   而非固定控制周期，避免低速时速度估算虚高。

  *   speed = (π/3 / pole_pairs) / Δt_real × (60 / 2π)  [rpm]

  *

  * 当长时间无扇区跳变时（超过 FOC_SECTOR_NO_CHANGE_THRESHOLD 个周期），

  * 认为电机已停止，速度衰减到零。

  *

  * 结果经过一阶低通滤波平滑处理。

  *

  * @param ctx        FOC 运行上下文（读写 hall_sector_prev, speed_filtered 等）

  * @param theta_e    当前电角度 (rad)

  * @param dt         控制周期 (s)，当前未使用，保留接口兼容

  * @param pole_pairs 极对数

  * @return           滤波后转速 (rpm)

  */

 float FOC_Observer_CalcSpeed(FOC_Context_t *ctx, float theta_e,

                               float dt, uint8_t pole_pairs);

 

 /**

  * @brief 根据当前速度和扇区角度，预测下一个控制周期的电角度

  *

  * 霍尔传感器仅在扇区跳变时更新角度（60°阶梯），在两次跳变之间

  * 角度保持不变，导致 Park/InvPark 变换使用的角度不连续，

  * 电流环无法正确解耦，电机产生电流声但不转。

  *

  * 本函数在扇区跳变时同步到跳变角度，在两次跳变之间根据

  * 当前估算速度线性外推，实现角度的连续变化。

  *

  * @param ctx        FOC 运行上下文（读写 theta_e_predicted, speed_filtered 等）

  * @param dt         控制周期 (s)

  * @param pole_pairs 极对数

  * @return           预测电角度 (rad)，范围 [0, 2π)

  */

/* Hybrid angle prediction:
 * during low-speed startup without reliable Hall edges, the returned angle is
 * the calibrated Hall sector center. With valid low-speed edges, the returned
 * angle follows linear Hall-sector interpolation. Above the PLL enter
 * threshold, the interpolation angle is used as the PLL input and the returned
 * angle is the PLL-smoothed output.
 */
 float FOC_Observer_PredictAngle(FOC_Context_t *ctx, float dt, uint8_t pole_pairs);

 

 /**

  * @brief 将三相电流 ADC 原始值换算为实际电流

  *

  * 换算公式：I = (ADC_raw - offset) × scale

  *   - offset: 零电流时的 ADC 采样值，驱动层通过 FOC_HAL_GetCurrentOffset() 提供

  *   - scale:  缓存于 calib->i_scale，初始化时从 FOC_HAL_GetCurrentScale() 获取

  *

  * @param raw      ADC 原始值

  * @param calib    标定参数（零点偏移）

  * @param[out] i   换算后的实际电流 (A)

  */

 void FOC_Observer_ParsePhaseCurrents(const FOC_PhaseCurrentRaw_t *raw,

                                       const FOC_CurrentCalib_t *calib,

                                       FOC_PhaseCurrent_t *i);

 

 /**

  * @brief 将母线电压 ADC 原始值换算为实际电压

  *

  * 换算公式：V = ADC_raw × scale

  *   - 母线电压采样为单极性，无中点偏移，不需要 offset

  *   - scale: 缓存于 calib->v_scale，初始化时从 FOC_HAL_GetVoltageScale() 获取

  *

  * @param raw      ADC 原始值

  * @param calib    标定参数（含 v_scale 缓存）

  * @return         母线电压 (V)

  */

 float FOC_Observer_ParseBusVoltage(uint32_t raw, const FOC_CurrentCalib_t *calib);

 

 #ifdef __cplusplus

 }

 #endif

 

 #endif /* FOC_OBSERVER_H */

 

 


