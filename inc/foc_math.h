
/**

 * @file foc_math.h

 * @brief FOC 数学工具 — sin/cos 查表插值、角度归一化、内联辅助

 *

 * 替代标准库 <math.h> 中的 cosf/sinf/fmodf/fabsf，

 * 适用于嵌入式实时中断环境：

 *   - sin/cos：256点查表 + 线性插值，单次计算约 20~30 个 CPU 周期

 *   - 角度归一化：位运算快速归一化到 [0, 2π)

 *   - fabsf：内联宏，无函数调用开销

 */

 

#ifndef FOC_MATH_H

#define FOC_MATH_H

 

#include "foc_config.h"

#include <stdint.h>

 

#ifdef __cplusplus

extern "C" {

#endif

 

/* ===================================================================

 *  内联辅助宏

 * =================================================================== */

 

/** 绝对值（替代 fabsf，无函数调用开销） */

#define FOC_FABS(x)     (((x) < 0.0f) ? -(x) : (x))

 

/** 限幅宏 */

#define FOC_CLAMP(x, lo, hi) (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))

 

/** 三相电流 KCL 约束：Ic = -(Ia + Ib) */

#define FOC_KCL_IC(ia, ib)  (-(ia) - (ib))

 

/* ===================================================================

 *  角度归一化

 * =================================================================== */

 

/**

 * @brief 将角度归一化到 [0, 2π)

 *

 * 使用减法循环替代 fmodf，无标准库依赖。

 * 适用于角度范围大致在 [-4π, +4π) 的常规场景。

 *

 * @param angle 输入角度 (rad)

 * @return      归一化角度 (rad)，范围 [0, 2π)

 */

static inline float FOC_NormalizeAngle(float angle)

{

    while (angle >= FOC_2PI) { angle -= FOC_2PI; }

    while (angle < 0.0f)    { angle += FOC_2PI; }

    return angle;

}

 

/* ===================================================================

 *  sin / cos 查表插值

 * =================================================================== */

 

/** 查表点数（2的幂，便于索引计算） */

#define FOC_SIN_TABLE_SIZE   256

 

/** 查表掩码 */

#define FOC_SIN_TABLE_MASK   (FOC_SIN_TABLE_SIZE - 1)

 

/**

 * @brief 初始化 sin 查找表

 *

 * 在 FOC_Init() 时调用一次，填充 256 点正弦表。

 * 也可在编译期静态初始化（节省启动时间）。

 */

void FOC_Math_InitTable(void);

 

/**

 * @brief 快速正弦计算（查表 + 线性插值）

 *

 * 精度：与 sinf 相比误差 < 0.0005（约 0.03°）

 * 速度：约 20~30 个 CPU 周期（Cortex-M4 @ 168MHz）

 *

 * @param angle 输入角度 (rad)，任意范围

 * @return      sin(angle)

 */

float FOC_FastSin(float angle);

 

/**

 * @brief 快速余弦计算（查表 + 线性插值）

 *

 * cos(x) = sin(x + π/2)，复用正弦表。

 *

 * @param angle 输入角度 (rad)，任意范围

 * @return      cos(angle)

 */

float FOC_FastCos(float angle);

 

/**

 * @brief 同时计算 sin 和 cos（查表 + 线性插值）

 *

 * 只需一次角度映射和一次查表，比分别调用快约 30%。

 * 在 Park / InvPark 变换中每周期调用一次。

 *

 * @param angle    输入角度 (rad)

 * @param[out] s   sin(angle)

 * @param[out] c   cos(angle)

 */

void FOC_FastSinCos(float angle, float *s, float *c);

 

#ifdef __cplusplus

}

#endif

 

#endif /* FOC_MATH_H */

 

 


