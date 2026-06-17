
/**

 * @file foc_observer.c

 * @brief FOC 观测器实现 — 霍尔传感器原始数据解析与角度/速度/电流换算

 *

 * 所有角度归一化使用 FOC_NormalizeAngle()（循环减法）替代 fmodf。

 * 所有绝对值使用 FOC_FABS() 宏替代 fabsf。

 */

 

 #include "foc_observer.h"

 #include "foc_math.h"

 #include "foc_config.h"

 #include "foc_hal_if.h"

 

 /* ===================================================================

  *  霍尔扇区查找表

  * =================================================================== */

 

 /**

  * 三路霍尔信号 H1/H2/H3 组合成3位编码 (H1<<2 | H2<<1 | H3)，

  * 用于查找扇区号。无效组合返回0。

  *

  * 编码值:  0   1   2   3   4   5   6   7

  * H1H2H3: 000 001 010 011 100 101 110 111

  * 扇区号:  0   6   4   5   2   1   3   0

  *          无效  6   4   5   2   1   3  无效

  */

 static const uint8_t s_hall_sector_lut[8] = {

     0,  /* 000: 无效 */

     6,  /* 001: 扇区6 (300°~360°) */

     4,  /* 010: 扇区4 (180°~240°) */

     5,  /* 011: 扇区5 (240°~300°) */

     2,  /* 100: 扇区2 (60°~120°) */

     1,  /* 101: 扇区1 (0°~60°) */

     3,  /* 110: 扇区3 (120°~180°) */

     0,  /* 111: 无效 */

 };

 

 /**

  * 扇区中心电角度查找表 (rad)

  * 扇区1: 30°,  扇区2: 90°,  扇区3: 150°

  * 扇区4: 210°, 扇区5: 270°, 扇区6: 330°

  */

 static const float s_hall_sector_angle_lut[7] = {

     0.0f,                                    /* 扇区0: 无效，不使用 */

     (30.0f  / 180.0f) * FOC_PI,              /* 扇区1: 30°  */

     (90.0f  / 180.0f) * FOC_PI,              /* 扇区2: 90°  */

     (150.0f / 180.0f) * FOC_PI,              /* 扇区3: 150° */

     (210.0f / 180.0f) * FOC_PI,              /* 扇区4: 210° */

     (270.0f / 180.0f) * FOC_PI,              /* 扇区5: 270° */

     (330.0f / 180.0f) * FOC_PI,              /* 扇区6: 330° */

 };

static float s_startup_predict_speed_rpm = 0.0f;

static float FOC_Observer_GetHallSyncAngle(uint8_t sector)
{
    if (sector >= 1U && sector <= 6U) {
        return FOC_NormalizeAngle(s_hall_sector_angle_lut[sector]
                                + FOC_HALL_ANGLE_OFFSET_RAD);
    }

    return 0.0f;
}

static uint8_t FOC_Observer_GetSectorStepCount(const FOC_Context_t *ctx,
                                                uint8_t prev_sector,
                                                uint8_t cur_sector)
{
    uint8_t steps;

    if ((prev_sector < 1U) || (prev_sector > 6U) ||
        (cur_sector < 1U) || (cur_sector > 6U) ||
        (prev_sector == cur_sector)) {
        return 1U;
    }

    if (ctx->direction == FOC_DIR_CCW) {
        steps = (uint8_t)((prev_sector + 6U - cur_sector) % 6U);
    } else {
        steps = (uint8_t)((cur_sector + 6U - prev_sector) % 6U);
    }

    return (steps == 0U) ? 1U : steps;
}



 /* ===================================================================

  *  观测器初始化

  * =================================================================== */

 

 void FOC_Observer_Init(FOC_Context_t *ctx)

 {

     ctx->hall_sector.sector       = 0;

     ctx->hall_sector.theta_e      = 0.0f;

     ctx->hall_sector_prev         = 0;

     ctx->theta_e_prev             = 0.0f;

     ctx->speed_raw                = 0.0f;

     ctx->speed_filtered           = 0.0f;

     ctx->timestamp_prev           = FOC_HAL_GetTimestampUs();

     ctx->sector_no_change_count   = 0U;

     ctx->theta_e_predicted        = 0.0f;
     s_startup_predict_speed_rpm   = 0.0f;

 

     /* Read current Hall state so first sector change is detected */

     FOC_HallRaw_t hall_init;

     FOC_HAL_GetHallRaw(&hall_init);

     FOC_Observer_HallRawToSector(&hall_init, &ctx->hall_sector);

     if (ctx->hall_sector.sector != 0U) {

         ctx->hall_sector_prev = ctx->hall_sector.sector;

         ctx->theta_e_predicted = ctx->hall_sector.theta_e;

     }

 }

 

 /* ===================================================================

  *  霍尔原始数据解析

  * =================================================================== */

 

 void FOC_Observer_HallRawToSector(const FOC_HallRaw_t *hall,

                                    FOC_HallSector_t *result)

 {

     /* 将三路电平组合为3位编码：H1为最高位，H3为最低位 */

     uint8_t code = (uint8_t)((hall->h1 << 2) | (hall->h2 << 1) | hall->h3);

 

     /* 查表获取扇区号 */

     result->sector = s_hall_sector_lut[code];

 

     /* 有效扇区号映射到中心电角度 */

     if (result->sector != 0U) {

         result->theta_e = FOC_Observer_GetHallSyncAngle(result->sector);

     }

     /* sector==0 时保留上一次 theta_e 不变，由调用方处理 */

 }

 

 float FOC_Observer_HallSectorToElecAngle(uint8_t sector)

 {

     if (sector >= 1U && sector <= 6U) {

         return FOC_Observer_GetHallSyncAngle(sector);

     }

     /* 无效扇区号，返回0（调用方应保留上一次角度） */

     return 0.0f;

 }

 

 /* ===================================================================

  *  速度估算

  * =================================================================== */

 

 float FOC_Observer_CalcSpeed(FOC_Context_t *ctx, float theta_e,

                               float dt, uint8_t pole_pairs)

 {

     float speed_rpm = ctx->speed_filtered;

 

     /* 检测扇区跳变 */

     uint8_t cur_sector  = ctx->hall_sector.sector;

     uint8_t prev_sector = ctx->hall_sector_prev;

 

     if (cur_sector != prev_sector && cur_sector != 0U) {

         /* 扇区跳变：使用实际时间戳计算真实时间间隔 */

         uint32_t ts_now  = FOC_HAL_GetTimestampUs();

 

        if (prev_sector != 0U) {

            /* Normal transition: compute speed from delta time */

            uint32_t dt_us   = ts_now - ctx->timestamp_prev;

            float dt_sec     = (float)dt_us * 1e-6f;

 

            if (pole_pairs > 0U && dt_sec > 1e-6f) {

                uint8_t sector_steps = FOC_Observer_GetSectorStepCount(ctx,
                                                                        prev_sector,
                                                                        cur_sector);
                float delta_theta_e = (FOC_PI / 3.0f) * (float)sector_steps;

                speed_rpm = (delta_theta_e / (float)pole_pairs) / dt_sec

                          * (60.0f / FOC_2PI);

 

                /* 限幅：防止异常跳变产生速度尖峰 */

                if (speed_rpm > FOC_SPEED_ESTIMATE_MAX_RPM) {

                    speed_rpm = FOC_SPEED_ESTIMATE_MAX_RPM;

                }

                if (speed_rpm < -FOC_SPEED_ESTIMATE_MAX_RPM) {

                    speed_rpm = -FOC_SPEED_ESTIMATE_MAX_RPM;

                }

            }

            ctx->hall_sector_dt_us = dt_us;

        }

        /* else: first valid sector from startup, skip speed calc */

 

         /* 更新跳变时间戳，为下次跳变计时 */

         ctx->timestamp_prev = ts_now;

        ctx->sector_no_change_count = 0U;

     } else {

        /* 无扇区跳变：保持上次跳变计算的速度，只有超过阈值周期时才衰减 */

        ctx->sector_no_change_count++;

        {
            uint32_t ts_now = FOC_HAL_GetTimestampUs();
            uint32_t stop_timeout_us = ctx->hall_sector_dt_us *
                                       FOC_HALL_STOP_TIMEOUT_RATIO;

            if (stop_timeout_us < FOC_HALL_STOP_TIMEOUT_MIN_US) {
                stop_timeout_us = FOC_HALL_STOP_TIMEOUT_MIN_US;
            }

            if ((ctx->hall_sector_dt_us != 0U) &&
                ((ts_now - ctx->timestamp_prev) >= stop_timeout_us)) {
                ctx->sector_no_change_count = FOC_SECTOR_NO_CHANGE_THRESHOLD;
            }
        }

        if (ctx->sector_no_change_count >= FOC_SECTOR_NO_CHANGE_THRESHOLD) {

            /* 长时间无跳变，电机已停止，速度衰减到零 */

            speed_rpm = 0.0f;
            ctx->speed_raw = 0.0f;
            ctx->speed_filtered = 0.0f;
            if (ctx->hall_sector.sector != 0U) {
                ctx->theta_e_predicted = ctx->hall_sector.theta_e;
            }
            ctx->hall_sector_prev = cur_sector;
            return 0.0f;

        }

    }

 

     /* 记录当前扇区 */

     ctx->hall_sector_prev = cur_sector;

 

     /* 一阶低通滤波 */

     ctx->speed_raw = speed_rpm;

     ctx->speed_filtered = FOC_SPEED_FILTER_ALPHA * speed_rpm

                         + (1.0f - FOC_SPEED_FILTER_ALPHA) * ctx->speed_filtered;

 

     return ctx->speed_filtered;

 }

 

 /* ===================================================================

  *  角度预测（扇区间插值）

  * =================================================================== */

 

 float FOC_Observer_PredictAngle(FOC_Context_t *ctx, float dt, uint8_t pole_pairs)

 {

     uint8_t cur_sector = ctx->hall_sector.sector;
     float speed_for_predict = ctx->speed_filtered;

     /* Always extrapolate by the real control interval first. */
     if ((ctx->speed_ref > 0.0f) &&
         (speed_for_predict < FOC_STARTUP_PREDICT_MAX_RPM)) {
         float startup_target = ctx->speed_ref;
         float ramp_step = FOC_STARTUP_PREDICT_RAMP_RPM_PER_S * dt;

         if (startup_target > FOC_STARTUP_PREDICT_MAX_RPM) {
             startup_target = FOC_STARTUP_PREDICT_MAX_RPM;
         }
         if (s_startup_predict_speed_rpm < FOC_STARTUP_PREDICT_START_RPM) {
             s_startup_predict_speed_rpm = FOC_STARTUP_PREDICT_START_RPM;
         }
         if (s_startup_predict_speed_rpm < startup_target) {
             s_startup_predict_speed_rpm += ramp_step;
             if (s_startup_predict_speed_rpm > startup_target) {
                 s_startup_predict_speed_rpm = startup_target;
             }
         } else if (s_startup_predict_speed_rpm > startup_target) {
             s_startup_predict_speed_rpm = startup_target;
         }
         speed_for_predict = s_startup_predict_speed_rpm;
     } else {
         s_startup_predict_speed_rpm = speed_for_predict;
     }

     if (speed_for_predict > 0.0f) {
         float omega_e = speed_for_predict * (FOC_2PI / 60.0f) * (float)pole_pairs;
         ctx->theta_e_predicted += omega_e * dt;
     }

 

     /* 扇区跳变时：软同步到跳变后的扇区中心角度，

      * 避免硬同步导致角度突变引发电流尖峰 */

     if (cur_sector != ctx->hall_sector_prev && cur_sector != 0U) {

         float target = ctx->hall_sector.theta_e;
         float sync_factor = FOC_ANGLE_SYNC_FACTOR;
         float sync_step;

         float diff = target - ctx->theta_e_predicted;

         /* 处理角度环绕 */

         if (diff > FOC_PI)  diff -= FOC_2PI;

         if (diff < -FOC_PI) diff += FOC_2PI;

         if (dt > ((float)FOC_CONTROL_LATE_PERIOD_US * 1.0e-6f)) {
             sync_factor = FOC_LATE_PERIOD_ANGLE_SYNC_FACTOR;
         }

         if (sync_factor > 1.0f) {
             sync_factor = 1.0f;
         } else if (sync_factor < 0.0f) {
             sync_factor = 0.0f;
         }

         sync_step = diff * sync_factor;

         if (FOC_ANGLE_SYNC_STEP_MAX_RAD > 0.0f) {
             if (sync_step > FOC_ANGLE_SYNC_STEP_MAX_RAD) {
                 sync_step = FOC_ANGLE_SYNC_STEP_MAX_RAD;
             } else if (sync_step < -FOC_ANGLE_SYNC_STEP_MAX_RAD) {
                 sync_step = -FOC_ANGLE_SYNC_STEP_MAX_RAD;
             }
         }

         ctx->theta_e_predicted += sync_step;

     } else {

        /* 角度外推：启动阶段 speed_filtered 尚未收敛，使用 speed_ref 驱动角度推进；

         * 速度收敛后自然切换到 speed_filtered，避免稳态角度超前 */

    }

 

     /* 归一化到 [0, 2π) */

     while (ctx->theta_e_predicted >= FOC_2PI) {

         ctx->theta_e_predicted -= FOC_2PI;

     }

     while (ctx->theta_e_predicted < 0.0f) {

         ctx->theta_e_predicted += FOC_2PI;

     }

 

     return ctx->theta_e_predicted;

 }

 

 /* ===================================================================

  *  电流/电压解析

  * =================================================================== */

 

 void FOC_Observer_ParsePhaseCurrents(const FOC_PhaseCurrentRaw_t *raw,

                                       const FOC_CurrentCalib_t *calib,

                                       FOC_PhaseCurrent_t *i)

 {

     /* I = (ADC_raw - offset) × scale */

     i->ia = ((float)raw->ia_raw - calib->ia_offset) * calib->i_scale;

     i->ib = ((float)raw->ib_raw - calib->ib_offset) * calib->i_scale;

     i->ic = ((float)raw->ic_raw - calib->ic_offset) * calib->i_scale;

 }

 

 float FOC_Observer_ParseBusVoltage(uint32_t raw, const FOC_CurrentCalib_t *calib)

 {

     /* V = ADC_raw × scale，母线电压为单极性采样，无中点偏移 */

     return (float)raw * calib->v_scale;

 }

 

 


