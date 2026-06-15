
/**

 * @file foc_math.c

 * @brief FOC 数学工具实现 — sin/cos 查表插值

 *

 * sin 查找表在初始化时使用标准库 sinf() 填充（仅调用一次），

 * 运行时所有 sin/cos 计算均通过查表完成，不依赖 <math.h>。

 */

 

#include "foc_math.h"

#include <math.h>   /* 仅用于 FOC_Math_InitTable() 初始化 */

 

/** 1/2π 常量，用于角度→索引映射 */

#define FOC_1_DIV_2PI     0.15915494309189533577f

 

/** sin 查找表 [0, 2π) → sin，共 FOC_SIN_TABLE_SIZE 个点 */

static float s_sin_table[FOC_SIN_TABLE_SIZE];

 

void FOC_Math_InitTable(void)

{

    uint16_t i;

    for (i = 0; i < FOC_SIN_TABLE_SIZE; i++) {

        /* 使用标准库仅在初始化时调用一次，运行时不再依赖 */

        float angle = (float)i / (float)FOC_SIN_TABLE_SIZE * FOC_2PI;

        s_sin_table[i] = sinf(angle);

    }

}

 

float FOC_FastSin(float angle)

{

    float norm;

    float idx_f;

    uint32_t idx0, idx1;

    float frac;

    float y0, y1;

 

    /* 归一化到 [0, 2π) */

    norm = angle * FOC_1_DIV_2PI;

    norm = norm - (float)(int32_t)norm;  /* 取小数部分 */

    if (norm < 0.0f) { norm += 1.0f; }

 

    /* 映射到表索引 [0, TABLE_SIZE) */

    idx_f = norm * (float)FOC_SIN_TABLE_SIZE;

    idx0  = (uint32_t)idx_f & FOC_SIN_TABLE_MASK;

    idx1  = (idx0 + 1U) & FOC_SIN_TABLE_MASK;

    frac  = idx_f - (float)(uint32_t)idx_f;

 

    /* 线性插值 */

    y0 = s_sin_table[idx0];

    y1 = s_sin_table[idx1];

 

    return y0 + frac * (y1 - y0);

}

 

float FOC_FastCos(float angle)

{

    /* cos(x) = sin(x + π/2) */

    return FOC_FastSin(angle + FOC_PI * 0.5f);

}

 

void FOC_FastSinCos(float angle, float *s, float *c)

{

    float norm;

    float idx_f;

    uint32_t idx0, idx1;

    float frac;

    float y0, y1;

 

    /* 归一化到 [0, 2π) */

    norm = angle * FOC_1_DIV_2PI;

    norm = norm - (float)(int32_t)norm;

    if (norm < 0.0f) { norm += 1.0f; }

 

    /* 映射到表索引 */

    idx_f = norm * (float)FOC_SIN_TABLE_SIZE;

    idx0  = (uint32_t)idx_f & FOC_SIN_TABLE_MASK;

    idx1  = (idx0 + 1U) & FOC_SIN_TABLE_MASK;

    frac  = idx_f - (float)(uint32_t)idx_f;

 

    /* sin：直接查表插值 */

    y0 = s_sin_table[idx0];

    y1 = s_sin_table[idx1];

    *s = y0 + frac * (y1 - y0);

 

    /* cos = sin(θ + π/2)：等价于表索引偏移 TABLE_SIZE/4 */

    {

        uint32_t cos_idx0 = (idx0 + (FOC_SIN_TABLE_SIZE / 4U)) & FOC_SIN_TABLE_MASK;

        uint32_t cos_idx1 = (cos_idx0 + 1U) & FOC_SIN_TABLE_MASK;

        y0 = s_sin_table[cos_idx0];

        y1 = s_sin_table[cos_idx1];

        *c = y0 + frac * (y1 - y0);

    }

}

 

 


