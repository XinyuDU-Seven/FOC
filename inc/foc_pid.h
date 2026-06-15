
/**

 * @file foc_pid.h

 * @brief FOC PID 控制器 — 位置式PID，含抗积分饱和

 */

 

#ifndef FOC_PID_H

#define FOC_PID_H

 

#include "foc_types.h"

 

#ifdef __cplusplus

extern "C" {

#endif

 

/**

 * @brief 用参数配置初始化 PID 控制器

 * @param pid   PID 运行时实例

 * @param params PID 参数配置（增益与输出限幅）

 */

void FOC_PID_Init(FOC_PID_t *pid, const FOC_PID_Params_t *params);

 

/**

 * @brief 执行一次 PID 计算并返回控制输出

 *

 * 位置式 PID：

 *   output = Kp·e + Ki·∫e·dt + Kd·(e - e_prev)/dt

 *

 * 抗积分饱和：当输出超出限幅时，停止积分累积。

 * 除法优化：调用方预计算 dt 的倒数传入，避免中断热路径中的硬件除法。

 *

 * @param pid       PID 运行时实例

 * @param error     当前误差（参考值 - 反馈值）

 * @param dt        时间步长 (s)

 * @param inv_dt    1/dt 时间步长倒数（预计算，避免运行时除法）

 * @return          PID 控制输出（已限幅至 [out_min, out_max]）

 */

float FOC_PID_Update(FOC_PID_t *pid, float error, float dt, float inv_dt);

 

/**

 * @brief 重置 PID 控制器内部状态（积分清零、历史误差清零）

 * @param pid PID 运行时实例

 */

void FOC_PID_Reset(FOC_PID_t *pid);

 

/**

 * @brief 在线修改 PID 参数（不影响积分等运行时状态）

 * @param pid    PID 运行时实例

 * @param params 新的 PID 参数

 */

void FOC_PID_SetParams(FOC_PID_t *pid, const FOC_PID_Params_t *params);

 

#ifdef __cplusplus

}

#endif

 

#endif /* FOC_PID_H */

 

 


