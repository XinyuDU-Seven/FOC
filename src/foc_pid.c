
/**

 * @file foc_pid.c

 * @brief FOC PID 控制器实现 — 位置式PID，含抗积分饱和

 *

 * 除法优化：微分项使用 inv_dt 替代 dt 做除法，

 * 中断热路径中避免硬件浮点除法（Cortex-M4 FPU 除法约14周期）。

 */

 

#include "foc_pid.h"

#include "foc_config.h"

 

void FOC_PID_Init(FOC_PID_t *pid, const FOC_PID_Params_t *params)

{

    pid->kp        = params->kp;

    pid->ki        = params->ki;

    pid->kd        = params->kd;

    pid->out_min   = params->out_min;

    pid->out_max   = params->out_max;

    pid->integral  = 0.0f;

    pid->prev_error = 0.0f;

}

 

float FOC_PID_Update(FOC_PID_t *pid, float error, float dt, float inv_dt)

{

    float p_term, i_term, d_term;

    float output;

 

    /* 比例项 */

    p_term = pid->kp * error;

 

    /* 积分项：先累积，后续根据抗饱和条件回退 */

    pid->integral += error * dt;

    i_term = pid->ki * pid->integral;

 

    /* 微分项：使用预计算的 inv_dt 避免除法 */

    d_term = pid->kd * (error - pid->prev_error) * inv_dt;

 

    /* 计算总输出 */

    output = p_term + i_term + d_term;

 

    /* 输出限幅 + 抗积分饱和 */

    if (output > pid->out_max) {

        output = pid->out_max;

#if FOC_PID_ANTI_WINDUP

        pid->integral -= error * dt;

#endif

    } else if (output < pid->out_min) {

        output = pid->out_min;

#if FOC_PID_ANTI_WINDUP

        pid->integral -= error * dt;

#endif

    }

 

    /* 保存本次误差 */

    pid->prev_error = error;

 

    return output;

}

 

void FOC_PID_Reset(FOC_PID_t *pid)

{

    pid->integral   = 0.0f;

    pid->prev_error = 0.0f;

}

 

void FOC_PID_SetParams(FOC_PID_t *pid, const FOC_PID_Params_t *params)

{

    pid->kp      = params->kp;

    pid->ki      = params->ki;

    pid->kd      = params->kd;

    pid->out_min = params->out_min;

    pid->out_max = params->out_max;

}

 

 


