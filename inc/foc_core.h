
/**

 * @file foc_core.h

 * @brief FOC 核心调度 — 主控制循环与状态机

 */

 

#ifndef FOC_CORE_H

#define FOC_CORE_H

 

#include "foc_types.h"

 

#ifdef __cplusplus

extern "C" {

#endif

 

/**

 * @brief 初始化 FOC 核心模块

 *

 * 初始化上下文、PID 控制器、观测器、保护模块。

 * 初始化后模块处于 FOC_STATE_IDLE 待机态。

 *

 * @param config 模块配置参数

 * @return FOC_OK 成功, FOC_ERR 失败

 */

int FOC_Core_Init(const FOC_Config_t *config);

 

/**

 * @brief 反初始化 FOC 核心模块

 * @return FOC_OK 成功

 */

int FOC_Core_DeInit(void);

 

/**

 * @brief 启动 FOC 控制

 *

 * 状态从 IDLE 转为 RUNNING，使能 PWM 输出。

 * @return FOC_OK 成功, FOC_BUSY 非IDLE态, FOC_FAULT 故障态

 */

int FOC_Core_Start(void);

 

/**

 * @brief 停止 FOC 控制

 *

 * 状态从 RUNNING 转为 IDLE，禁用 PWM 输出，PID 复位。

 * @return FOC_OK 成功

 */

int FOC_Core_Stop(void);

 

/**

 * @brief FOC 主控制循环

 *

 * 在定时器中断中以控制频率调用（如 10kHz）。

 * 完成一次完整的 FOC 控制周期：

 *   1. 读取传感器 → 2. 观测器 → 3. Clarke → 4. Park

 *   → 5. 速度环 → 6. 电流环 → 7. 逆Park → 8. SVPWM

 *   → 9. 输出PWM → 10. 保护检测

 *

 * 仅在 RUNNING 状态下执行算法，其他状态为空操作。

 */

void FOC_Core_MainLoop(void);

 

/**

 * @brief 获取 FOC 运行上下文指针（只读访问）

 * @return 上下文指针

 */

const FOC_Context_t *FOC_Core_GetContext(void);

 

/**

 * @brief 设置目标转速

 * @param rpm 目标转速 (rpm)，负值表示反转

 * @return FOC_OK 成功

 */

int FOC_Core_SetSpeedRef(float rpm);

 

/**

 * @brief 设置旋转方向

 * @param dir 旋转方向

 * @return FOC_OK 成功

 */

int FOC_Core_SetDirection(FOC_Dir_e dir);

 

/**

 * @brief 在线修改速度环 PID 参数

 * @param pid PID 参数

 * @return FOC_OK 成功

 */

int FOC_Core_SetSpeedPID(const FOC_PID_Params_t *pid);

 

/**

 * @brief 在线修改电流环 PID 参数

 * @param pid_d d轴PID参数

 * @param pid_q q轴PID参数

 * @return FOC_OK 成功

 */

int FOC_Core_SetCurrentPID(const FOC_PID_Params_t *pid_d, const FOC_PID_Params_t *pid_q);

 

/**

 * @brief 清除故障

 * @return FOC_OK 成功, FOC_FAULT 仍有未清除故障

 */

int FOC_Core_ClearFault(void);

 

#ifdef __cplusplus

}

#endif

 

#endif /* FOC_CORE_H */

 

 


