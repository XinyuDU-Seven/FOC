
/**

 * @file foc_transform.h

 * @brief FOC 坐标变换 — Clarke / Park / InversePark

 */

 

#ifndef FOC_TRANSFORM_H

#define FOC_TRANSFORM_H

 

#include "foc_types.h"

 

#ifdef __cplusplus

extern "C" {

#endif

 

/**

 * @brief Clarke 变换：三相静止坐标系 → 两相静止坐标系

 *

 *  Iα = Ia

 *  Iβ = (Ia + 2·Ib) / √3

 *

 * @param i_abc   三相电流输入 (Ia, Ib, Ic)

 * @param[out] i_ab αβ 坐标系电流输出 (Iα, Iβ)

 */

void FOC_Clarke(const FOC_PhaseCurrent_t *i_abc, FOC_AlphaBeta_t *i_ab);

 

/**

 * @brief Park 变换：两相静止坐标系 → 两相旋转坐标系

 *

 *  Id =  Iα·cos(θ) + Iβ·sin(θ)

 *  Iq = -Iα·sin(θ) + Iβ·cos(θ)

 *

 * @param i_ab    αβ 坐标系电流

 * @param theta   电角度 (rad)

 * @param[out] i_dq dq 旋转坐标系电流输出 (Id, Iq)

 */

void FOC_Park(const FOC_AlphaBeta_t *i_ab, float theta, FOC_DQ_t *i_dq);

 

/**

 * @brief 逆 Park 变换：两相旋转坐标系 → 两相静止坐标系

 *

 *  Vα = Vd·cos(θ) - Vq·sin(θ)

 *  Vβ = Vd·sin(θ) + Vq·cos(θ)

 *

 * @param v_dq    dq 旋转坐标系电压

 * @param theta   电角度 (rad)

 * @param[out] v_ab αβ 坐标系电压输出 (Vα, Vβ)

 */

void FOC_InvPark(const FOC_DQ_t *v_dq, float theta, FOC_AlphaBeta_t *v_ab);

 

#ifdef __cplusplus

}

#endif

 

#endif /* FOC_TRANSFORM_H */

 

 


