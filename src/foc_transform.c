
/**

 * @file foc_transform.c

 * @brief FOC 坐标变换实现 — Clarke / Park / InversePark

 *

 * 使用 foc_math.h 的快速 sin/cos 查表替代标准库 cosf/sinf，

 * Park 和 InvPark 共享一次 FOC_FastSinCos 调用。

 * 所有函数改为输出指针模式，避免栈上结构体拷贝。

 */

 

#include "foc_transform.h"

#include "foc_math.h"

#include "foc_config.h"

 

void FOC_Clarke(const FOC_PhaseCurrent_t *i_abc, FOC_AlphaBeta_t *i_ab)

{

    /* Iα = Ia */

    i_ab->alpha = i_abc->ia;

 

    /* Iβ = (Ia + 2·Ib) / √3 */

    i_ab->beta = FOC_1_DIV_SQRT3 * (i_abc->ia + 2.0f * i_abc->ib);

}

 

void FOC_Park(const FOC_AlphaBeta_t *i_ab, float theta, FOC_DQ_t *i_dq)

{

    float sin_theta, cos_theta;

 

    FOC_FastSinCos(theta, &sin_theta, &cos_theta);

 

    /* Id =  Iα·cos(θ) + Iβ·sin(θ) */

    i_dq->d = i_ab->alpha * cos_theta + i_ab->beta * sin_theta;

 

    /* Iq = -Iα·sin(θ) + Iβ·cos(θ) */

    i_dq->q = -i_ab->alpha * sin_theta + i_ab->beta * cos_theta;

}

 

void FOC_InvPark(const FOC_DQ_t *v_dq, float theta, FOC_AlphaBeta_t *v_ab)

{

    float sin_theta, cos_theta;

 

    FOC_FastSinCos(theta, &sin_theta, &cos_theta);

 

    /* Vα = Vd·cos(θ) - Vq·sin(θ) */

    v_ab->alpha = v_dq->d * cos_theta - v_dq->q * sin_theta;

 

    /* Vβ = Vd·sin(θ) + Vq·cos(θ) */

    v_ab->beta = v_dq->d * sin_theta + v_dq->q * cos_theta;

}

 

 


