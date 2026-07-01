
/**

 * @file foc_hal_if.h

 * @brief FOC 电机控制模块 — 硬件抽象接口定义

 *

 * 本文件声明 FOC 模块对底层驱动层的需求契约。

 * FOC 模块通过这些接口获取传感器原始数据、输出控制信号，

 * 不直接操作任何硬件寄存器。

 *

 * 设计原则：

 *   - 驱动层仅提供原始硬件数据（霍尔电平、ADC原始值等），

 *     不做物理量解析与换算

 *   - 数据解析（原始值→角度/电流等物理量）由 FOC 模块内部的

 *     observer / port 层负责

 *

 * 所有函数由 foc_port.c 实现适配，调用实际的驱动层接口。

 * 移植时仅需修改 foc_port.c，FOC 算法代码无需任何变更。

 */

 

#ifndef FOC_HAL_IF_H

#define FOC_HAL_IF_H

 

#include "foc_types.h"

 

#ifdef __cplusplus

extern "C" {

#endif

 

/* ===================================================================

 *  初始化接口

 * =================================================================== */

 

/**

 * @brief 初始化驱动层

 *

 * 完成底层硬件初始化（ADC校准、霍尔GPIO配置、PWM定时器配置等）。

 * FOC_Init() 内部调用此函数，应用层无需单独调用。

 *

 * @return 0 成功, -1 失败

 */

int  FOC_HAL_Init(void);

void FOC_HAL_SelectMotor(uint8_t motor_id);

uint8_t FOC_HAL_GetSelectedMotor(void);

uint8_t FOC_HAL_GetActivePhysicalMotor(void);

 

/* ===================================================================

 *  霍尔传感器接口

 * =================================================================== */

 

/**

 * @brief 获取霍尔传感器三路原始数字电平

 *

 * 驱动层直接读取三个 GPIO 引脚电平，填充 h1/h2/h3 字段

 * （0 或 1），不做任何解析换算。

 * FOC 观测器负责将三路电平组合解析为扇区号和电角度。

 *

 * @param[out] hall 霍尔传感器三路电平值

 */

void FOC_HAL_GetHallRaw(FOC_HallRaw_t *hall);

uint8_t FOC_HAL_GetHallEvent(FOC_HallRaw_t *hall,
                             uint32_t *timestamp_us,
                             uint32_t *seq);

 

/* ===================================================================

 *  电流采样接口

 * =================================================================== */

 

/**

 * @brief 获取三相电流 ADC 原始采样值

 * @param[out] raw 三相电流 ADC 原始值，由 FOC observer 层负责转换为实际电流

 *

 * 驱动层返回 ADC 寄存器原始值，FOC observer 层完成偏移校正与比例换算。

 */

void     FOC_HAL_GetPhaseCurrentsRaw(FOC_PhaseCurrentRaw_t *raw);

 

/**

 * @brief 获取电流采样 ADC → 实际电流的比例系数 (A/LSB)

 *

 * scale 由硬件电路参数决定，计算公式：

 *   scale = V_ref / ((2^N - 1) × R_shunt × G)

 *

 * 其中 N 为 ADC 位数，R_shunt 为采样电阻 (Ω)，G 为运放增益。

 * 驱动层根据实际硬件参数计算返回，FOC 模块不感知硬件细节。

 *

 * @return 电流比例系数 (A/LSB)

 */

float    FOC_HAL_GetCurrentScale(void);

 

/**

 * @brief 获取三相电流 ADC 零点偏移

 *

 * offset 为零电流时 ADC 采样值，由驱动层提供。

 * 驱动层可在上电时自动标定（多次采样取均值），或使用已知的硬件中点值。

 *

 * @param[out] offset 三相 ADC 零点偏移

 */

void     FOC_HAL_GetCurrentOffset(FOC_CurrentCalib_t *offset);

 

/* ===================================================================

 *  PWM 控制接口

 * =================================================================== */

 

/**

 * @brief 设置三相 PWM 占空比

 * @param dA A相占空比，范围 [0.0, 1.0]

 * @param dB B相占空比，范围 [0.0, 1.0]

 * @param dC C相占空比，范围 [0.0, 1.0]

 */

void     FOC_HAL_SetDutyCycle(float dA, float dB, float dC);

 

/**

 * @brief 使能 PWM 输出

 *

 * 调用后 PWM 开始输出，电机可能开始旋转。

 * FOC_Start() 内部调用此函数。

 */

void     FOC_HAL_EnablePWM(void);

 

/**

 * @brief 禁用 PWM 输出

 *

 * 调用后 PWM 输出关闭，电机自由停转。

 * FOC_Stop() 及故障保护内部调用此函数。

 */

void     FOC_HAL_DisablePWM(void);

 

/* ===================================================================

 *  系统接口

 * =================================================================== */

 

/**

 * @brief 获取控制环路频率

 * @return 控制频率 (Hz)，例如 10000 表示 10kHz 控制周期

 *

 * 此值由驱动层根据定时器中断配置确定，

 * FOC 模块据此计算速度观测器的差分时间步长。

 */

uint32_t FOC_HAL_GetControlFreq(void);

 

/**

 * @brief 获取微秒级时间戳

 * @return 当前时间戳 (us)

 *

 * 用于速度估算中的时间差计算和保护计时。

 */

uint32_t FOC_HAL_GetTimestampUs(void);

 

/**

 * @brief 获取母线电压 ADC 原始采样值

 * @return ADC 原始值，由 FOC observer 层转换为实际电压

 */

uint32_t FOC_HAL_GetBusVoltageRaw(void);

 

/**

 * @brief 获取母线电压 ADC → 实际电压的比例系数 (V/LSB)

 *

 * scale 由硬件分压电路参数决定，计算公式：

 *   scale = V_ref / ((2^N - 1) × R_ratio)

 *

 * 其中 N 为 ADC 位数，R_ratio 为分压比 (R1+R2)/R2。

 * 驱动层根据实际硬件参数计算返回。

 *

 * @return 电压比例系数 (V/LSB)

 */

float    FOC_HAL_GetVoltageScale(void);

 

/* ===================================================================

 *  临界区保护接口

 * =================================================================== */

 

/**

 * @brief 进入临界区（关中断）

 *

 * FOC 主循环在中断中运行，应用层 API 在主循环中调用，

 * 两者可能并发访问 speed_ref / PID 参数等共享数据。

 * 驱动层实现关中断/开中断，FOC 模块在访问共享变量时调用。

 */

void FOC_HAL_EnterCritical(void);

 

/**

 * @brief 退出临界区（开中断）

 */

void FOC_HAL_ExitCritical(void);

 

#ifdef __cplusplus

}

#endif

 

#endif /* FOC_HAL_IF_H */

 

 


