
/**

 * @file foc_svpwm.h

 * @brief FOC SVPWM 调制 — 空间矢量PWM扇区判断与占空比计算

 */

 

#ifndef FOC_SVPWM_H

#define FOC_SVPWM_H

 

#include "foc_types.h"

 

#ifdef __cplusplus

extern "C" {

#endif

 

/**

 * @brief SVPWM 调制计算

 *

 * 将 αβ 坐标系电压矢量转换为三相 PWM 占空比。

 * 内部完成：

 *   1. 判断电压矢量所在扇区 (1~6)

 *   2. 计算相邻基本矢量的作用时间 (T1, T2)

 *   3. 计算三相占空比 (Da, Db, Dc)

 *

 * @param v_alpha  α轴电压分量 (V)

 * @param v_beta   β轴电压分量 (V)

 * @param v_bus    母线电压 (V)，用于归一化

 * @param[out] duty_a  A相占空比 [0, 1]

 * @param[out] duty_b  B相占空比 [0, 1]

 * @param[out] duty_c  C相占空比 [0, 1]

 */

void FOC_SVPWM_Calculate(float v_alpha, float v_beta, float v_bus,

                          float *duty_a, float *duty_b, float *duty_c);

 

#ifdef __cplusplus

}

#endif

 

#endif /* FOC_SVPWM_H */

 

 


