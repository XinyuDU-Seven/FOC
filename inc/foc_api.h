/**
 * @file foc_api.h
 * @brief FOC 电机控制模块 — 对应用层统一接口
 *
 * 应用层只需 #include "foc_api.h" 即可使用 FOC 模块全部功能。
 * 所有接口内部委托给 foc_core 实现，本文件是唯一的对外入口。
 */

#ifndef FOC_API_H
#define FOC_API_H

#include "foc_types.h"
#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 *  初始化与启停
 * =================================================================== */

/**
 * @brief 初始化 FOC 模块
 *
 * 传入电机参数与 PID 配置，初始化内部状态。
 * 初始化后模块处于 IDLE 待机态，PWM 关闭。
 *
 * @param config 模块配置（电机参数 + PID 参数）
 * @return FOC_OK 成功, FOC_ERR 参数无效
 */
int  FOC_Init(const FOC_Config_t *config);

/**
 * @brief 反初始化 FOC 模块，释放资源
 * @return FOC_OK 成功
 */
int  FOC_DeInit(void);

/**
 * @brief 启动 FOC 控制
 *
 * PWM 使能输出，PID 复位，开始闭环控制。
 * 仅在 IDLE 态可启动。
 *
 * @return FOC_OK 成功, FOC_BUSY 非IDLE态, FOC_FAULT 故障态
 */
int  FOC_Start(void);

/**
 * @brief 停止 FOC 控制
 *
 * PWM 关闭，PID 复位，速度参考归零，回到 IDLE 态。
 *
 * @return FOC_OK 成功
 */
int  FOC_Stop(void);

/* ===================================================================
 *  运行时控制
 * =================================================================== */

/**
 * @brief 设定目标转速
 * @param rpm 目标转速 (rpm)，正值正转，负值反转
 * @return FOC_OK 成功
 */
int  FOC_SetSpeedRef(float rpm);

/**
 * @brief Set direct d/q current references.
 * @param id d-axis current reference (A)
 * @param iq q-axis current reference (A), positive forward, negative reverse
 * @return FOC_OK on success
 */
int  FOC_SetCurrentRef(float id, float iq);

/**
 * @brief Set I/F current reference with forced angle advance.
 * @param iq q-axis current reference (A), positive forward, negative reverse
 * @param rpm forced mechanical speed reference (rpm), signed like iq
 * @return FOC_OK on success
 */
int  FOC_SetIFRef(float iq, float rpm);

/**
 * @brief 设定旋转方向
 * @param dir 旋转方向 (FOC_DIR_CW / FOC_DIR_CCW)
 * @return FOC_OK 成功
 */
int  FOC_SetDirection(FOC_Dir_e dir);

/**
 * @brief 获取当前实际转速
 * @param[out] rpm 实际转速 (rpm)
 * @return FOC_OK 成功
 */
int  FOC_GetSpeed(float *rpm);

/**
 * @brief 获取当前运行状态
 * @param[out] state 运行状态
 * @return FOC_OK 成功
 */
int  FOC_GetState(FOC_State_e *state);

/* ===================================================================
 *  参数在线调节
 * =================================================================== */

/**
 * @brief 在线修改速度环 PID 参数
 * @param pid PID 参数（仅修改增益和限幅，不影响积分状态）
 * @return FOC_OK 成功, FOC_ERR 参数无效
 */
int  FOC_SetSpeedPID(const FOC_PID_Params_t *pid);

/**
 * @brief 在线修改电流环 PID 参数
 * @param pid PID 参数（d轴和q轴使用相同参数）
 * @return FOC_OK 成功, FOC_ERR 参数无效
 */
int  FOC_SetCurrentPID(const FOC_PID_Params_t *pid);

/* ===================================================================
 *  故障与保护
 * =================================================================== */

/**
 * @brief 获取当前故障码
 * @param[out] fault 故障码（位域，可同时存在多种故障）
 * @return FOC_OK 成功
 */
int  FOC_GetFaultCode(FOC_Fault_e *fault);

/**
 * @brief 清除故障，恢复到 IDLE 待机态
 *
 * 仅在 FAULT 态有效。清除后需重新调用 FOC_Start() 启动。
 *
 * @return FOC_OK 成功
 */
int  FOC_ClearFault(void);

/* ===================================================================
 *  主循环入口（供驱动层定时器中断回调）
 * =================================================================== */

/**
 * @brief FOC 主控制循环入口
 *
 * 由驱动层定时器中断以控制频率（如 10kHz）周期性调用。
 * 应用层不应直接调用此函数。
 */
void FOC_MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_API_H */
