/**
 * @file foc_svpwm.c
 * @brief FOC SVPWM modulation implementation.
 */

#include "foc_svpwm.h"

#include "foc_math.h"
#include "foc_config.h"

void FOC_SVPWM_Calculate(float v_alpha, float v_beta, float v_bus,
                         float *duty_a, float *duty_b, float *duty_c)
{
    float va;
    float vb;
    float vc;
    float v_max;
    float v_min;
    float v_zero;
    float v_abs_max;
    float v_limit;

    if (v_bus < 0.1f) {
        *duty_a = 0.0f;
        *duty_b = 0.0f;
        *duty_c = 0.0f;
        return;
    }

    /*
     * Continuous SVPWM by zero-sequence injection.
     *
     * Convert the alpha/beta voltage vector to three phase-voltage commands,
     * inject a common-mode offset, then map the centered phase voltages to
     * PWM duties. This avoids the sector-table discontinuity seen during
     * Hall/SVPWM sector boundary crossings.
     */
    va = v_alpha;
    vb = -0.5f * v_alpha + FOC_SQRT3_DIV2 * v_beta;
    vc = -0.5f * v_alpha - FOC_SQRT3_DIV2 * v_beta;

    v_max = va;
    if (vb > v_max) {
        v_max = vb;
    }
    if (vc > v_max) {
        v_max = vc;
    }

    v_min = va;
    if (vb < v_min) {
        v_min = vb;
    }
    if (vc < v_min) {
        v_min = vc;
    }

    v_zero = -0.5f * (v_max + v_min);
    va += v_zero;
    vb += v_zero;
    vc += v_zero;

    v_abs_max = FOC_FABS(va);
    if (FOC_FABS(vb) > v_abs_max) {
        v_abs_max = FOC_FABS(vb);
    }
    if (FOC_FABS(vc) > v_abs_max) {
        v_abs_max = FOC_FABS(vc);
    }

    v_limit = 0.5f * v_bus * FOC_SVPWM_MODULATION_MAX;
    if ((v_abs_max > v_limit) && (v_abs_max > 0.001f)) {
        float scale = v_limit / v_abs_max;
        va *= scale;
        vb *= scale;
        vc *= scale;
    }

    *duty_a = FOC_CLAMP(0.5f + va / v_bus, 0.0f, 1.0f);
    *duty_b = FOC_CLAMP(0.5f + vb / v_bus, 0.0f, 1.0f);
    *duty_c = FOC_CLAMP(0.5f + vc / v_bus, 0.0f, 1.0f);
}
