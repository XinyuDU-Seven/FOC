
/**

 * @file foc_protection.h

 * @brief FOC 保护逻辑 — 过流/过压/欠压/堵转/霍尔传感器异常检测

 */

 

#ifndef FOC_PROTECTION_H

#define FOC_PROTECTION_H

 

#include "foc_types.h"

 

#ifdef __cplusplus

extern "C" {

#endif

 

/** 保护阈值配置（运行时可调） */

typedef struct {

    float    overcurrent_a;      /**< 过流阈值 (A) */

    float    overvoltage_v;      /**< 过压阈值 (V) */

    float    undervoltage_v;     /**< 欠压阈值 (V) */

    float    stall_speed_err;    /**< 堵转速度误差阈值 (rpm) */

    uint32_t stall_count;        /**< 堵转连续计数阈值 */

    uint32_t hall_invalid_count; /**< 霍尔无效扇区连续计数阈值 */

} FOC_Protection_Threshold_t;

 

/**

 * @brief 获取默认保护阈值（基于 foc_config.h 中的宏）

 * @param[out] th 阈值结构体

 */

void FOC_Protection_GetDefaultThreshold(FOC_Protection_Threshold_t *th);

 

/**

 * @brief 设置保护阈值

 * @param th 新的保护阈值

 */

void FOC_Protection_SetThreshold(const FOC_Protection_Threshold_t *th);

 

/**

 * @brief 执行一次保护检测

 *

 * 检测项：

 *   - 过流：三相电流绝对值峰值超过阈值

 *   - 过压/欠压：母线电压超出安全范围

 *   - 堵转：速度误差持续过大

 *   - 霍尔异常：无效扇区持续超过阈值

 *

 * 检测到故障时，自动将 ctx->fault 置位。

 *

 * @param ctx   FOC 运行上下文

 * @param v_bus 当前母线电压 (V)

 * @return      当前故障码（0 表示无故障）

 */

FOC_Fault_e FOC_Protection_Check(FOC_Context_t *ctx, float v_bus);

 

/**

 * @brief 清除故障状态

 *

 * 清除 ctx 中的故障码和堵转计数器，

 * 但不会自动恢复运行状态（由状态机决定）。

 *

 * @param ctx FOC 运行上下文

 */

void FOC_Protection_ClearFault(FOC_Context_t *ctx);

 

#ifdef __cplusplus

}

#endif

 

#endif /* FOC_PROTECTION_H */

 

 


