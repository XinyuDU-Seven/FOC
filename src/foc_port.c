
/**

 * @file foc_port.c

 * @brief FOC 硬件抽象接口移植适配层 — 模板实现

 *

 * 本文件实现 foc_hal_if.h 中声明的所有 FOC_HAL_* 函数。

 * 移植时需根据实际硬件平台修改此文件，对接底层驱动。

 *

 * 设计原则：

 *   - 本文件只返回驱动层的原始数据（霍尔电平、ADC原始值），

 *     不做物理量解析与换算

 *   - 数据解析由 FOC 观测器（foc_observer）负责，

 *     使用 FOC_Config_t 中的标定参数完成换算

 *   - 这样解析逻辑与硬件解耦，标定参数可在线修改，便于调校

 */

 

#include "foc_hal_if.h"

#include "foc_config.h"

#include "Icu.h"

#include "Adc.h"

#include "pwm.h"

#include "ProjectCfg.h"

#include <stdint.h>

 

#define MOTOR_HALL_IO_PIN_U_0 P14_01 /* 水平电机HU(V0.2模具件硬件: P14_01, V0.1模具件硬件: P14_01) */

#define MOTOR_HALL_IO_PIN_V_0 P16_01 /* 水平电机HV(V0.2模具件硬件: P16_01, V0.1模具件硬件: P15_02) */

#define MOTOR_HALL_IO_PIN_W_0 P16_02 /* 水平电机HW(V0.2模具件硬件: P16_02, V0.1模具件硬件: P16_00) */

#define MOTOR_HALL_IO_PIN_U_1 P10_00 /* 高调电机HU(V0.2模具件硬件: P10_00, V0.1模具件硬件: P10_01) */

#define MOTOR_HALL_IO_PIN_V_1 P12_04 /* 高调电机HV(V0.2模具件硬件: P12_04, V0.1模具件硬件: P14_06) */

#define MOTOR_HALL_IO_PIN_W_1 P14_02 /* 高调电机HW(V0.2模具件硬件: P14_02, V0.1模具件硬件: P14_02) */

 

typedef struct {

    uint8_t unHA;

    uint8_t unHB;

    uint8_t unHC;

}HallState_t;

 

HallState_t gstHallState[2] = {0};

 

//霍尔回调处理函数

void FOC_Motor1_HallCallback(void){

    //目前仅支持单个电机

    /* 读取霍尔传感器A/B/C状态 */

    gstHallState[0].unHA = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_U_0);

    gstHallState[0].unHB = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_V_0);

    gstHallState[0].unHC = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_W_0);

}

 

//霍尔回调处理函数

void FOC_Motor2_HallCallback(void){

    //目前仅支持单个电机

    /* 读取霍尔传感器A/B/C状态 */

    gstHallState[1].unHA = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_U_1);

    gstHallState[1].unHB = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_V_1);

    gstHallState[1].unHC = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_W_1);

}

 

/* ===================================================================

 *  驱动层初始化

 * =================================================================== */

int FOC_HAL_Init(void)

{

    ICU_SetUserIsr(MOTOR_HALL_IO_PIN_U_0, FOC_Motor1_HallCallback);

    ICU_SetUserIsr(MOTOR_HALL_IO_PIN_V_0, FOC_Motor1_HallCallback);

    ICU_SetUserIsr(MOTOR_HALL_IO_PIN_W_0, FOC_Motor1_HallCallback);

 

    ICU_SetUserIsr(MOTOR_HALL_IO_PIN_U_1, FOC_Motor2_HallCallback);

    ICU_SetUserIsr(MOTOR_HALL_IO_PIN_V_1, FOC_Motor2_HallCallback);

    ICU_SetUserIsr(MOTOR_HALL_IO_PIN_W_1, FOC_Motor2_HallCallback);

   

    /* 使能霍尔信号通知，V0.2硬件需要主动使能*/

    Icu_EnableNotification(MOTOR_HALL_IO_PIN_U_0);

    Icu_EnableNotification(MOTOR_HALL_IO_PIN_V_0);

    Icu_EnableNotification(MOTOR_HALL_IO_PIN_W_0);

 

    Icu_EnableNotification(MOTOR_HALL_IO_PIN_U_1);

    Icu_EnableNotification(MOTOR_HALL_IO_PIN_V_1);

    Icu_EnableNotification(MOTOR_HALL_IO_PIN_W_1);

 

    FOC_Motor1_HallCallback(); //初始时手动调用一次

    FOC_Motor2_HallCallback();

    return 0;

}

 

/* ===================================================================

 *  霍尔传感器接口实现

 * =================================================================== */

 

void FOC_HAL_GetHallRaw(FOC_HallRaw_t *hall)

{

    //目前仅支持单个电机

    /* Read GPIO directly in the control loop; do not depend on Hall IRQ cache. */

    hall->h1 = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_U_0);

    hall->h2 = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_V_0);

    hall->h3 = (uint8_t)Dio_ReadChannel(MOTOR_HALL_IO_PIN_W_0);

    gstHallState[0].unHA = hall->h1;

    gstHallState[0].unHB = hall->h2;

    gstHallState[0].unHC = hall->h3;

}

 

/* ===================================================================

 *  电流采样接口实现

 *  输出参数：三相电流adc采样值

 * =================================================================== */

void FOC_HAL_GetPhaseCurrentsRaw(FOC_PhaseCurrentRaw_t *raw)

{

    uint16_t pBuffer[4] = {0};

 

    /* 获取ADC值 */

    int unId = 0;

    Adc_GetBldcFocCurrentVoltage(unId, pBuffer);

    raw->ia_raw = pBuffer[1];

    raw->ib_raw = pBuffer[2];

    raw->ic_raw = pBuffer[3];

}

 

/*

*  返回三相电流 ADC 零点偏移

*/

void FOC_HAL_GetCurrentOffset(FOC_CurrentCalib_t *offset)

{

    offset->ia_offset = 2048.0f;

    offset->ib_offset = 2048.0f;

    offset->ic_offset = 2048.0f;

}

 

/* ===================================================================

 *  PWM 控制接口实现

 * =================================================================== */

void FOC_HAL_SetDutyCycle(float dA, float dB, float dC)

{

    uint8_t unId = 0;

    uint32_t unDutyA = dA * 10000;

    uint32_t unDutyB = dB * 10000;

    uint32_t unDutyC = dC * 10000;

    Pwm_BldcFocPwmToSVPwm(unId, unDutyA, unDutyB, unDutyC);

}

 

void FOC_HAL_EnablePWM(void)

{

    /*

     * TODO: 使能 PWM 输出

     *

     * 示例：

     *   TIM_PWM->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

     *   TIM_PWM->BDTR |= TIM_BDTR_MOE;

     * 目前无需使能

     */

}

 

void FOC_HAL_DisablePWM(void)

{

    /*

     * 禁用 PWM 输出

     */

    uint8_t unId = 0;

    Pwm_BldcFocPwmToSVPwm(unId, 0, 0, 0); /* 先直接停机，再改状态 */

}

 

/* ===================================================================

 *  系统接口实现

 * =================================================================== */

 

uint32_t FOC_HAL_GetControlFreq(void)

{

    /*

     * 返回控制环路频率 // 10kHz

     */

    return FOC_CONTROL_FREQ_HZ;

}

 

uint32_t FOC_HAL_GetTimestampUs(void)

{

    return (uint32_t)GetSystemTime();//实际接口是uint64_t,此接口有溢出风险。

}

 

//获取电机母线电压adc采样值

uint32_t FOC_HAL_GetBusVoltageRaw(void)

{

    uint16_t pBuffer[4] = {0};

 

    /* 获取ADC值 */

    int unId = 0;

    Adc_GetBldcFocCurrentVoltage(unId, pBuffer);

    uint32_t unUdc= pBuffer[0];

    return unUdc;

}

 

#define FOC_BSP_ADC_CURRENT_RESOLUTION    4095.0f   /* 电流ADC 12位 */

#define FOC_BSP_ADC_VOLTAGE_RESOLUTION    4095.0f   /* 电压ADC 12位 */

#define FOC_BSP_ADC_VREF                  5.0f      /* ADC参考电压 */

#define MOTOR_ADC_AMPF_0                  12.50f    /* 0号电机ADC电流放大倍率 */

#define FOC_BSP_ADC_VOLTAGE_DIVIDER_RATIO 13.0f/3.0f /* 电压分压比 */

 

/*

*  返回电流采样 ADC → 实际电流的比例系数 (A/LSB)

*

*  计算公式：scale = V_ref / ((2^N - 1) × R_shunt × G)

*   N:        ADC 位数

*   V_ref:    ADC 参考电压 (V)

*   R_shunt:  采样电阻 (Ω)

*   G:        运放增益 (V/V)

*/

float FOC_HAL_GetCurrentScale(void)

{

    /* 计算相电流: I = 放大系数 * 5 * (ADC  - 2048) / 4095 ; */

    return MOTOR_ADC_AMPF_0 * FOC_BSP_ADC_VREF / FOC_BSP_ADC_CURRENT_RESOLUTION;

}

 

/*

* 返回母线电压 ADC → 实际电压的比例系数 (V/LSB)

*/

float FOC_HAL_GetVoltageScale(void)

{

    /* 计算母线电压: Udc = ADC * (Vref/4095) * 分压比 */

    /* Udc = (pBuffer[0] / 4095.0 * 5.0 * 13.0 / 3.0) */

    return FOC_BSP_ADC_VREF * FOC_BSP_ADC_VOLTAGE_DIVIDER_RATIO / FOC_BSP_ADC_VOLTAGE_RESOLUTION;

}

 

/* ===================================================================

 *  临界区保护实现

 * =================================================================== */

 

void FOC_HAL_EnterCritical(void)

{

    /*

     * TODO: 关中断，保护 FOC 共享变量不被 ISR 与主线程并发访问

     *

     * 示例（ARM Cortex-M）：

     *   __disable_irq();

     *   // 或保存 BASEPRI 实现可嵌套：

     *   // uint32_t primask = __get_PRIMASK();

     *   // __disable_irq();

     *   // return primask;  // 需配合 ExitCritical 使用

     *

     * 示例（FreeRTOS）：

     *   taskENTER_CRITICAL();

     */

}

 

void FOC_HAL_ExitCritical(void)

{

    /*

     * TODO: 开中断

     *

     * 示例（ARM Cortex-M）：

     *   __enable_irq();

     *

     * 示例（FreeRTOS）：

     *   taskEXIT_CRITICAL();

     */

}

 

 


