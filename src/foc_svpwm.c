
/**

 * @file foc_svpwm.c

 * @brief FOC SVPWM 调制实现

 *

 * 基于扇区判断的标准 SVPWM 算法：

 *   1. 由 Vα、Vβ 计算扇区判断变量 U1/U2/U3

 *   2. 由扇区号计算 T1/T2 作用时间

 *   3. 计算 Ta/Tb/Tc 并映射为三相占空比

 */

 

#include "foc_svpwm.h"

#include "foc_math.h"

#include "foc_config.h"

 

 

void FOC_SVPWM_Calculate(float v_alpha, float v_beta, float v_bus,

                          float *duty_a, float *duty_b, float *duty_c)

{

    float u1, u2, u3;

    float t1, t2, t0;

    float ta, tb, tc;

    uint8_t sector = 0;

 

    /* 防止母线电压为零导致除零 */

    if (v_bus < 0.1f) {

        *duty_a = 0.0f;

        *duty_b = 0.0f;

        *duty_c = 0.0f;

        return;

    }

 

    /* ---- 第1步：归一化到母线电压 ---- */

    /* 乘以 √3 / Vbus 将电压归一化，方便后续扇区判断 */

    float v_ref = FOC_SQRT3 / v_bus;

    float va = v_alpha * v_ref;

    float vb = v_beta * v_ref;

 

    /* ---- 第2步：计算扇区判断变量 ---- */

    u1 = vb;

    u2 = (FOC_SQRT3_DIV2 * va - 0.5f * vb);

    u3 = (-FOC_SQRT3_DIV2 * va - 0.5f * vb);

 

    /* ---- 第3步：确定扇区号 (1~6) ---- */

    if (u1 > 0.0f) sector |= 0x01;

    if (u2 > 0.0f) sector |= 0x02;

    if (u3 > 0.0f) sector |= 0x04;

 

    /* sector 编码 → 扇区号映射表

     * 0x01→2, 0x02→6, 0x03→1, 0x04→4, 0x05→3, 0x06→5, 0x07→无效

     */

    switch (sector) {

        case 1: sector = 2; break;

        case 2: sector = 6; break;

        case 3: sector = 1; break;

        case 4: sector = 4; break;

        case 5: sector = 3; break;

        case 6: sector = 5; break;

        default: sector = 0; break;

    }

 

    /* ---- 第4步：计算相邻矢量作用时间 T1, T2 ---- */

    switch (sector) {

        case 1:

            t1 =  u2;

            t2 =  u1;

            break;

        case 2:

            t1 = -u2;

            t2 = -u3;

            break;

        case 3:

            t1 =  u1;

            t2 =  u3;

            break;

        case 4:

            t1 = -u1;

            t2 = -u2;

            break;

        case 5:

            t1 =  u3;

            t2 =  u2;

            break;

        case 6:

            t1 = -u3;

            t2 = -u1;

            break;

        default:

            t1 = 0.0f;

            t2 = 0.0f;

            break;

    }

 

    /* 过调制处理：T1+T2 > 1 时等比例缩小 */

    if (t1 + t2 > 1.0f) {

        float scale = 1.0f / (t1 + t2);

        t1 *= scale;

        t2 *= scale;

    }

 

    /* 调制系数上限限制 */

    if ((t1 + t2) > FOC_SVPWM_MODULATION_MAX) {

        float scale = FOC_SVPWM_MODULATION_MAX / (t1 + t2);

        t1 *= scale;

        t2 *= scale;

    }

 

    /* 零矢量时间 */

    t0 = 1.0f - t1 - t2;

 

    /* ---- 第5步：计算三相切换时间 Ta, Tb, Tc ---- */

    switch (sector) {

        case 1:

            ta = t1 + t2 + t0 / 2.0f;

            tb = t2 + t0 / 2.0f;

            tc = t0 / 2.0f;

            break;

        case 2:

            ta = t1 + t0 / 2.0f;

            tb = t1 + t2 + t0 / 2.0f;

            tc = t0 / 2.0f;

            break;

        case 3:

            ta = t0 / 2.0f;

            tb = t1 + t2 + t0 / 2.0f;

            tc = t2 + t0 / 2.0f;

            break;

        case 4:

            ta = t0 / 2.0f;

            tb = t1 + t0 / 2.0f;

            tc = t1 + t2 + t0 / 2.0f;

            break;

        case 5:

            ta = t2 + t0 / 2.0f;

            tb = t0 / 2.0f;

            tc = t1 + t2 + t0 / 2.0f;

            break;

        case 6:

            ta = t1 + t2 + t0 / 2.0f;

            tb = t0 / 2.0f;

            tc = t1 + t0 / 2.0f;

            break;

        default:

            ta = 0.5f;

            tb = 0.5f;

            tc = 0.5f;

            break;

    }

 

    /* ---- 第6步：输出占空比，限幅到 [0, 1] ---- */

    *duty_a = FOC_CLAMP(ta, 0.0f, 1.0f);

    *duty_b = FOC_CLAMP(tb, 0.0f, 1.0f);

    *duty_c = FOC_CLAMP(tc, 0.0f, 1.0f);

}

 

 


