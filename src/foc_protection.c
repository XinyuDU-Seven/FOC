
/**

 * @file foc_protection.c

 * @brief FOC 保护逻辑实现

 */

 

#include "foc_protection.h"

#include "foc_math.h"

#include "foc_hal_if.h"

#include "foc_config.h"

#include <stddef.h>

 

/** 当前生效的保护阈值（模块内部静态变量） */

static FOC_Protection_Threshold_t s_threshold;

 

/** 霍尔无效扇区连续计数 */

static uint32_t s_hall_invalid_counter;

 

void FOC_Protection_GetDefaultThreshold(FOC_Protection_Threshold_t *th)

{

    th->overcurrent_a      = FOC_OVERCURRENT_THRESHOLD_A;

    th->overvoltage_v      = FOC_OVERVOLTAGE_THRESHOLD_V;

    th->undervoltage_v     = FOC_UNDERVOLTAGE_THRESHOLD_V;

    th->stall_speed_err    = FOC_STALL_SPEED_ERROR_RPM;

    th->stall_count        = FOC_STALL_COUNT_THRESHOLD;

    th->hall_invalid_count = FOC_HALL_INVALID_COUNT_THRESHOLD;

}

 

void FOC_Protection_SetThreshold(const FOC_Protection_Threshold_t *th)

{

    s_threshold = *th;

}

 

FOC_Fault_e FOC_Protection_Check(FOC_Context_t *ctx, float v_bus)

{

    FOC_Fault_e fault = FOC_FAULT_NONE;

 

    /* ---- 过流检测：三相电流绝对值峰值 ---- */

    float i_peak = FOC_FABS(ctx->i_abc.ia);

    float ib_abs = FOC_FABS(ctx->i_abc.ib);

    float ic_abs = FOC_FABS(ctx->i_abc.ic);

    if (ib_abs > i_peak) i_peak = ib_abs;

    if (ic_abs > i_peak) i_peak = ic_abs;

    ctx->current_peak = i_peak;

 

    if (i_peak > s_threshold.overcurrent_a) {

        fault |= FOC_FAULT_OVERCURRENT;

    }

 

    /* ---- 过压检测 ---- */

    if (v_bus > s_threshold.overvoltage_v) {

        fault |= FOC_FAULT_OVERVOLTAGE;

    }

 

    /* ---- 欠压检测 ---- */

    if (v_bus < s_threshold.undervoltage_v) {

        fault |= FOC_FAULT_UNDERVOLTAGE;

    }

    /* ---- 启动失败/堵转检测：有速度指令但反馈速度长期接近 0 ---- */
    {
        float speed_ref_abs = FOC_FABS(ctx->speed_ref_ctrl);
        float speed_fdb_abs = FOC_FABS(ctx->speed_fdb);
        float speed_err = FOC_FABS(ctx->speed_ref_ctrl - ctx->speed_fdb);

        if ((ctx->state == FOC_STATE_RUNNING) &&
            (speed_ref_abs >= FOC_STALL_MIN_SPEED_REF_RPM) &&
            (speed_fdb_abs <= FOC_STALL_MAX_FEEDBACK_RPM) &&
            (speed_err > s_threshold.stall_speed_err)) {
            ctx->stall_counter++;
            if (ctx->stall_counter >= s_threshold.stall_count) {
                fault |= FOC_FAULT_STALL;
            }
        } else {
            ctx->stall_counter = 0U;
        }
    }


#if 0

    /* ---- 堵转检测：速度误差持续超过阈值 ---- */

    float speed_err = FOC_FABS(ctx->speed_ref_ctrl - ctx->speed_fdb);

    if (ctx->state == FOC_STATE_RUNNING && speed_err > s_threshold.stall_speed_err) {

        ctx->stall_counter++;

        if (ctx->stall_counter >= s_threshold.stall_count) {

            fault |= FOC_FAULT_STALL;

        }

    } else {

        ctx->stall_counter = 0;

    }

 

 

    /* ---- 霍尔传感器异常检测：无效扇区持续计数 ---- */

    if (ctx->hall_sector.sector == 0U) {

        s_hall_invalid_counter++;

        if (s_hall_invalid_counter >= s_threshold.hall_invalid_count) {

            // fault |= FOC_FAULT_HALL;

        }

    } else {

        s_hall_invalid_counter = 0;

    }

#endif

    /* 合并故障码到上下文 */

    if (fault != FOC_FAULT_NONE) {

        ctx->fault |= fault;

    }

 

    return ctx->fault;

}

 

void FOC_Protection_ClearFault(FOC_Context_t *ctx)

{

    ctx->fault = FOC_FAULT_NONE;

    ctx->stall_counter = 0U;

    s_hall_invalid_counter = 0U;

}

 

 


