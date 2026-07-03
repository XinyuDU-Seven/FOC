
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

#ifdef __ICCARM__
#define FOC_OBSERVER_DEBUG_ROOT __root
#else
#define FOC_OBSERVER_DEBUG_ROOT
#endif
 

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

#define FOC_OBSERVER_MOTOR_COUNT 2U

static float s_startup_predict_speed_rpm_store[FOC_OBSERVER_MOTOR_COUNT] = {
    0.0f,
    0.0f
};
static FOC_Dir_e s_predict_direction_store[FOC_OBSERVER_MOTOR_COUNT] = {
    FOC_DIR_CW,
    FOC_DIR_CW
};
static uint8_t s_startup_sync_edge_count_store[FOC_OBSERVER_MOTOR_COUNT] = {
    0U,
    0U
};

static uint8_t FOC_Observer_GetMotorIndex(void)
{
    uint8_t motor = FOC_HAL_GetSelectedMotor();

    return (motor < FOC_OBSERVER_MOTOR_COUNT) ? motor : 0U;
}

#define s_startup_predict_speed_rpm \
    (s_startup_predict_speed_rpm_store[FOC_Observer_GetMotorIndex()])
#define s_predict_direction \
    (s_predict_direction_store[FOC_Observer_GetMotorIndex()])
#define s_startup_sync_edge_count \
    (s_startup_sync_edge_count_store[FOC_Observer_GetMotorIndex()])
FOC_OBSERVER_DEBUG_ROOT volatile uint32_t g_foc_observer_direction_reset_count = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint32_t g_foc_observer_no_edge_decay_count = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint32_t g_foc_observer_no_edge_elapsed_us = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint16_t g_foc_observer_no_edge_speed_limit_rpm = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint16_t g_foc_observer_no_edge_decay_min_speed_rpm =
    FOC_HALL_NO_EDGE_DECAY_MIN_SPEED_RPM;
FOC_OBSERVER_DEBUG_ROOT volatile uint8_t  g_foc_observer_no_edge_active = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint32_t g_foc_observer_resync_count = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_observer_resync_diff_mrad = 0;
FOC_OBSERVER_DEBUG_ROOT volatile uint8_t  g_foc_observer_startup_ref_active = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint16_t g_foc_observer_predict_speed_rpm = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint16_t g_foc_observer_startup_release_rpm =
    (uint16_t)FOC_STARTUP_PREDICT_RELEASE_RPM;
FOC_OBSERVER_DEBUG_ROOT volatile uint8_t  g_foc_observer_startup_sync_active = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint8_t  g_foc_observer_startup_sync_boost_active = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint16_t g_foc_observer_startup_sync_edge_count = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint8_t  g_foc_observer_recovery_sync_active = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_observer_sync_step_mrad = 0;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_observer_sync_diff_mrad = 0;
FOC_OBSERVER_DEBUG_ROOT volatile uint8_t  g_foc_observer_startup_pre_edge_clamp_active = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint32_t g_foc_observer_startup_pre_edge_clamp_count = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_observer_startup_pre_edge_clamp_step_mrad = 0;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_observer_startup_pre_edge_clamp_diff_mrad = 0;
FOC_OBSERVER_DEBUG_ROOT volatile uint8_t  g_foc_observer_no_edge_angle_clamp_active = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile uint32_t g_foc_observer_no_edge_angle_clamp_count = 0U;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_observer_no_edge_angle_diff_mrad = 0;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_observer_no_edge_angle_limit_mrad = 0;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_observer_no_edge_angle_step_mrad = 0;
FOC_OBSERVER_DEBUG_ROOT volatile int16_t  g_foc_hall_angle_offset_mrad =
    (int16_t)(FOC_HALL_ANGLE_OFFSET_RAD * 1000.0f);

static float FOC_Observer_NormalizeAngleDiff(float diff)
{
    while (diff > FOC_PI) {
        diff -= FOC_2PI;
    }
    while (diff < -FOC_PI) {
        diff += FOC_2PI;
    }

    return diff;
}

static float FOC_Observer_GetHallAngleTrim(uint8_t sector)
{
    static const float trim_lut[7] = {
        0.0f,
        FOC_HALL_ANGLE_TRIM_S1_RAD,
        FOC_HALL_ANGLE_TRIM_S2_RAD,
        FOC_HALL_ANGLE_TRIM_S3_RAD,
        FOC_HALL_ANGLE_TRIM_S4_RAD,
        FOC_HALL_ANGLE_TRIM_S5_RAD,
        FOC_HALL_ANGLE_TRIM_S6_RAD,
    };

    if (sector >= 1U && sector <= 6U) {
        return trim_lut[sector];
    }

    return 0.0f;
}

static float FOC_Observer_GetHallAngleOffset(void)
{
    return (float)g_foc_hall_angle_offset_mrad * 0.001f;
}

static float FOC_Observer_GetHallSyncAngle(uint8_t sector)
{
    if (sector >= 1U && sector <= 6U) {
        return FOC_NormalizeAngle(s_hall_sector_angle_lut[sector]
                                + FOC_Observer_GetHallAngleTrim(sector)
                                + FOC_Observer_GetHallAngleOffset());
    }

    return 0.0f;
}

static FOC_Dir_e FOC_Observer_GetOppositeDir(FOC_Dir_e direction)
{
    return (direction == FOC_DIR_CCW) ? FOC_DIR_CW : FOC_DIR_CCW;
}

static FOC_Dir_e FOC_Observer_GetConfiguredForwardHallDir(void)
{
#if FOC_FORWARD_HALL_DIR
    return FOC_DIR_CCW;
#else
    return FOC_DIR_CW;
#endif
}

static FOC_Dir_e FOC_Observer_GetHallMotionDir(const FOC_Context_t *ctx)
{
    FOC_Dir_e forward_dir = FOC_Observer_GetConfiguredForwardHallDir();

    return (ctx->direction == FOC_DIR_CCW)
         ? FOC_Observer_GetOppositeDir(forward_dir)
         : forward_dir;
}

static float FOC_Observer_GetHallEntryAngle(uint8_t sector, FOC_Dir_e hall_dir)
{
    if (sector >= 1U && sector <= 6U) {
        float edge_offset = (hall_dir == FOC_DIR_CCW)
                          ? (FOC_PI / 6.0f)
                          : -(FOC_PI / 6.0f);

        return FOC_NormalizeAngle(s_hall_sector_angle_lut[sector]
                                + edge_offset
                                + FOC_Observer_GetHallAngleTrim(sector)
                                + FOC_Observer_GetHallAngleOffset());
    }

    return 0.0f;
}

#if FOC_STARTUP_PRE_EDGE_CLAMP_ENABLE
static float FOC_Observer_GetHallExitAngle(uint8_t sector, FOC_Dir_e hall_dir)
{
    if (sector >= 1U && sector <= 6U) {
        float edge_offset = (hall_dir == FOC_DIR_CCW)
                          ? -(FOC_PI / 6.0f)
                          :  (FOC_PI / 6.0f);

        return FOC_NormalizeAngle(s_hall_sector_angle_lut[sector]
                                + edge_offset
                                + FOC_Observer_GetHallAngleTrim(sector)
                                + FOC_Observer_GetHallAngleOffset());
    }

    return 0.0f;
}
#endif

static void FOC_Observer_ClampStartupPreEdgeAngle(FOC_Context_t *ctx,
                                                  uint8_t cur_sector,
                                                  FOC_Dir_e hall_dir)
{
#if FOC_STARTUP_PRE_EDGE_CLAMP_ENABLE
    float margin = FOC_STARTUP_PRE_EDGE_MARGIN_RAD;
    float step_max = FOC_STARTUP_PRE_EDGE_CLAMP_STEP_MAX_RAD;
    float exit_angle;
    float diff;
    float target;
    float step;
    uint8_t clamp_needed = 0U;

    if ((ctx == 0) ||
        (cur_sector == 0U) ||
        (ctx->hall_sector_dt_us != 0U) ||
        (cur_sector != ctx->hall_sector_prev) ||
        (g_foc_observer_startup_ref_active == 0U)) {
        return;
    }

    if (margin < 0.0f) {
        margin = -margin;
    }
    if (margin > (FOC_PI / 3.0f)) {
        margin = FOC_PI / 3.0f;
    }
    if (step_max < 0.0f) {
        step_max = -step_max;
    }

    exit_angle = FOC_Observer_GetHallExitAngle(cur_sector, hall_dir);
    diff = FOC_Observer_NormalizeAngleDiff(ctx->theta_e_predicted -
                                           exit_angle);

    if (hall_dir == FOC_DIR_CCW) {
        if (diff < -margin) {
            target = FOC_NormalizeAngle(exit_angle - margin);
            clamp_needed = 1U;
        } else {
            target = ctx->theta_e_predicted;
        }
    } else {
        if (diff > margin) {
            target = FOC_NormalizeAngle(exit_angle + margin);
            clamp_needed = 1U;
        } else {
            target = ctx->theta_e_predicted;
        }
    }

    g_foc_observer_startup_pre_edge_clamp_diff_mrad =
        (int16_t)(diff * 1000.0f);

    if (clamp_needed == 0U) {
        return;
    }

    step = FOC_Observer_NormalizeAngleDiff(target - ctx->theta_e_predicted);
    if (step_max > 0.0f) {
        if (step > step_max) {
            step = step_max;
        } else if (step < -step_max) {
            step = -step_max;
        }
    }

    ctx->theta_e_predicted =
        FOC_NormalizeAngle(ctx->theta_e_predicted + step);
    g_foc_observer_startup_pre_edge_clamp_step_mrad =
        (int16_t)(step * 1000.0f);
    g_foc_observer_startup_pre_edge_clamp_active = 1U;
    g_foc_observer_startup_pre_edge_clamp_count++;
#else
    (void)ctx;
    (void)cur_sector;
    (void)hall_dir;
#endif
}

static float FOC_Observer_GetHallEdgeSyncAngle(const FOC_Context_t *ctx,
                                                uint8_t sector,
                                                float omega_e,
                                                float advance_max)
{
    uint32_t now_us = FOC_HAL_GetTimestampUs();
    uint32_t age_us = (ctx->hall_sector_timestamp_us != 0U)
                    ? (now_us - ctx->hall_sector_timestamp_us)
                    : 0U;
    FOC_Dir_e hall_dir = FOC_Observer_GetHallMotionDir(ctx);
    float target = FOC_Observer_GetHallEntryAngle(sector, hall_dir);
    float advance = omega_e * ((float)age_us * 1.0e-6f);

    if (advance < 0.0f) {
        advance = 0.0f;
    }
    if (advance_max < 0.0f) {
        advance_max = 0.0f;
    }
    if (advance > advance_max) {
        advance = advance_max;
    }

    if (hall_dir == FOC_DIR_CCW) {
        return FOC_NormalizeAngle(target - advance);
    }

    return FOC_NormalizeAngle(target + advance);
}

float FOC_Observer_HallEdgeSyncAngle(const FOC_Context_t *ctx,
                                     uint8_t sector,
                                     float omega_e,
                                     float advance_max)
{
    if ((ctx == 0) || (sector == 0U)) {
        return 0.0f;
    }

    return FOC_Observer_GetHallEdgeSyncAngle(ctx, sector, omega_e, advance_max);
}

static uint8_t FOC_Observer_GetSectorStepCount(const FOC_Context_t *ctx,
                                                uint8_t prev_sector,
                                                uint8_t cur_sector)
{
    uint8_t cw_steps;
    uint8_t ccw_steps;

    (void)ctx;
    if ((prev_sector < 1U) || (prev_sector > 6U) ||
        (cur_sector < 1U) || (cur_sector > 6U) ||
        (prev_sector == cur_sector)) {
        return 1U;
    }

    cw_steps = (uint8_t)((cur_sector + 6U - prev_sector) % 6U);
    ccw_steps = (uint8_t)((prev_sector + 6U - cur_sector) % 6U);

    if (cw_steps == 0U) {
        cw_steps = 6U;
    }
    if (ccw_steps == 0U) {
        ccw_steps = 6U;
    }

    return (cw_steps < ccw_steps) ? cw_steps : ccw_steps;
}

static uint8_t FOC_Observer_HallStepMatchesDirection(const FOC_Context_t *ctx,
                                                      uint8_t prev_sector,
                                                      uint8_t cur_sector)
{
    uint8_t cw_steps;
    uint8_t ccw_steps;

    if ((prev_sector < 1U) || (prev_sector > 6U) ||
        (cur_sector < 1U) || (cur_sector > 6U) ||
        (prev_sector == cur_sector)) {
        return 1U;
    }

    cw_steps = (uint8_t)((cur_sector + 6U - prev_sector) % 6U);
    ccw_steps = (uint8_t)((prev_sector + 6U - cur_sector) % 6U);

    if (FOC_Observer_GetHallMotionDir(ctx) == FOC_DIR_CCW) {
        return (ccw_steps <= cw_steps) ? 1U : 0U;
    }

    return (cw_steps <= ccw_steps) ? 1U : 0U;
}
static uint8_t FOC_Observer_NoEdgeOverdue(const FOC_Context_t *ctx,
                                          uint8_t pole_pairs,
                                          uint32_t *elapsed_us,
                                          float *speed_limit_rpm)
{
    uint32_t now_us;
    uint32_t elapsed;
    uint32_t decay_start_us;
    uint32_t limit_elapsed_us;
    float decay_start_f;
    float ratio = FOC_HALL_NO_EDGE_DECAY_START_RATIO;
    float limit_rpm;
    float speed_abs;
    float min_speed_rpm;

    if ((ctx->hall_sector_dt_us == 0U) ||
        (ctx->hall_sector.sector == 0U) ||
        (pole_pairs == 0U)) {
        return 0U;
    }

    now_us = FOC_HAL_GetTimestampUs();
    elapsed = now_us - ctx->timestamp_prev;
    if (elapsed == 0U) {
        return 0U;
    }

    speed_abs = ctx->speed_filtered;
    if (speed_abs < 0.0f) {
        speed_abs = -speed_abs;
    }
    min_speed_rpm = (float)g_foc_observer_no_edge_decay_min_speed_rpm;
    if ((min_speed_rpm > 0.0f) && (speed_abs < min_speed_rpm)) {
        return 0U;
    }

    if (ratio < 1.0f) {
        ratio = 1.0f;
    }
    decay_start_f = (float)ctx->hall_sector_dt_us * ratio;
    decay_start_us = (uint32_t)(decay_start_f + 0.5f);
    if (decay_start_us < ctx->hall_sector_dt_us) {
        decay_start_us = ctx->hall_sector_dt_us;
    }

    if (elapsed <= decay_start_us) {
        return 0U;
    }

    limit_elapsed_us = elapsed - decay_start_us + ctx->hall_sector_dt_us;
    if (limit_elapsed_us == 0U) {
        return 0U;
    }

    limit_rpm = 10000000.0f /
                ((float)pole_pairs * (float)limit_elapsed_us);

    if (limit_rpm < 0.0f) {
        limit_rpm = 0.0f;
    } else if (limit_rpm > FOC_SPEED_ESTIMATE_MAX_RPM) {
        limit_rpm = FOC_SPEED_ESTIMATE_MAX_RPM;
    }

    if (limit_rpm >= speed_abs) {
        return 0U;
    }

    if (elapsed_us != 0) {
        *elapsed_us = elapsed;
    }
    if (speed_limit_rpm != 0) {
        *speed_limit_rpm = limit_rpm;
    }

    return 1U;
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

     ctx->speed_ref_ctrl           = 0.0f;

     ctx->speed_filtered           = 0.0f;

     ctx->speed_ctrl_fdb           = 0.0f;

     ctx->timestamp_prev           = FOC_HAL_GetTimestampUs();
     ctx->hall_sector_timestamp_us = ctx->timestamp_prev;

     ctx->sector_no_change_count   = 0U;

     ctx->theta_e_predicted        = 0.0f;
     s_startup_predict_speed_rpm   = 0.0f;
     s_predict_direction           = FOC_Observer_GetHallMotionDir(ctx);
     s_startup_sync_edge_count     = 0U;
     g_foc_observer_direction_reset_count = 0U;
     g_foc_observer_no_edge_decay_count = 0U;
     g_foc_observer_no_edge_elapsed_us = 0U;
     g_foc_observer_no_edge_speed_limit_rpm = 0U;
     g_foc_observer_no_edge_decay_min_speed_rpm =
         FOC_HALL_NO_EDGE_DECAY_MIN_SPEED_RPM;
     g_foc_observer_no_edge_active = 0U;
     g_foc_observer_resync_count = 0U;
     g_foc_observer_resync_diff_mrad = 0;
     g_foc_observer_startup_sync_boost_active = 0U;
     g_foc_observer_startup_sync_edge_count = 0U;
     g_foc_observer_startup_pre_edge_clamp_active = 0U;
     g_foc_observer_startup_pre_edge_clamp_count = 0U;
     g_foc_observer_startup_pre_edge_clamp_step_mrad = 0;
     g_foc_observer_startup_pre_edge_clamp_diff_mrad = 0;
     g_foc_observer_no_edge_angle_clamp_active = 0U;
     g_foc_observer_no_edge_angle_clamp_count = 0U;
     g_foc_observer_no_edge_angle_diff_mrad = 0;
     g_foc_observer_no_edge_angle_limit_mrad = 0;
     g_foc_observer_no_edge_angle_step_mrad = 0;

 

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

     g_foc_observer_no_edge_active = 0U;

 

     /* 检测扇区跳变 */

     uint8_t cur_sector  = ctx->hall_sector.sector;

     uint8_t prev_sector = ctx->hall_sector_prev;

 

     if (cur_sector != prev_sector && cur_sector != 0U) {

         /* 扇区跳变：使用实际时间戳计算真实时间间隔 */

         uint32_t ts_now  = (ctx->hall_sector_timestamp_us != 0U)
                          ? ctx->hall_sector_timestamp_us
                          : FOC_HAL_GetTimestampUs();

 

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
            uint32_t no_edge_elapsed_us = 0U;
            float no_edge_limit_rpm = 0.0f;

            if (FOC_Observer_NoEdgeOverdue(ctx, pole_pairs,
                                           &no_edge_elapsed_us,
                                           &no_edge_limit_rpm) != 0U) {
                if (speed_rpm > no_edge_limit_rpm) {
                    speed_rpm = no_edge_limit_rpm;
                }
                if (ctx->speed_filtered > no_edge_limit_rpm) {
                    ctx->speed_filtered = no_edge_limit_rpm;
                }
                g_foc_observer_no_edge_active = 1U;
                g_foc_observer_no_edge_decay_count++;
                g_foc_observer_no_edge_elapsed_us = no_edge_elapsed_us;
                g_foc_observer_no_edge_speed_limit_rpm =
                    (uint16_t)((no_edge_limit_rpm > 65535.0f)
                             ? 65535U
                             : no_edge_limit_rpm);
            }
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
     float omega_e = 0.0f;
     uint32_t no_edge_elapsed_us = 0U;
     float no_edge_limit_rpm = 0.0f;
     float startup_release_rpm = (float)g_foc_observer_startup_release_rpm;
     float startup_release_err_rpm = FOC_STARTUP_PREDICT_RELEASE_ERR_RPM;
     float release_blend_rate_rpm_s =
         FOC_STARTUP_PREDICT_RELEASE_BLEND_RPM_PER_S;
     float release_blend_done_rpm =
         FOC_STARTUP_PREDICT_RELEASE_BLEND_DONE_RPM;
     float speed_filtered_abs = FOC_FABS(ctx->speed_filtered);
     float speed_ref_ctrl_abs = FOC_FABS(ctx->speed_ref_ctrl);
     float startup_track_err_rpm = speed_ref_ctrl_abs - speed_filtered_abs;
     FOC_Dir_e hall_dir;
     uint8_t use_startup_ref_predict;
     uint8_t startup_release_blend_active = 0U;
     uint8_t no_edge_overdue = FOC_Observer_NoEdgeOverdue(ctx, pole_pairs,
                                                               &no_edge_elapsed_us,
                                                               &no_edge_limit_rpm);
     hall_dir = FOC_Observer_GetHallMotionDir(ctx);
     if (startup_release_rpm < FOC_STARTUP_PREDICT_START_RPM) {
         startup_release_rpm = FOC_STARTUP_PREDICT_START_RPM;
     }
     if (startup_release_err_rpm < 0.0f) {
         startup_release_err_rpm = 0.0f;
     }
     if (startup_track_err_rpm < 0.0f) {
         startup_track_err_rpm = 0.0f;
     }
     if (release_blend_rate_rpm_s < 0.0f) {
         release_blend_rate_rpm_s = -release_blend_rate_rpm_s;
     }
     if (release_blend_done_rpm < 0.0f) {
         release_blend_done_rpm = -release_blend_done_rpm;
     }
     use_startup_ref_predict =
         ((ctx->hall_sector_dt_us == 0U) ||
          (speed_filtered_abs < startup_release_rpm) ||
          ((speed_filtered_abs < FOC_STARTUP_PREDICT_MAX_RPM) &&
           (speed_ref_ctrl_abs > startup_release_rpm) &&
           (startup_track_err_rpm > startup_release_err_rpm)))
         ? 1U : 0U;
     g_foc_observer_startup_ref_active = 0U;
     g_foc_observer_startup_sync_active = 0U;
     g_foc_observer_startup_sync_boost_active = 0U;
     g_foc_observer_startup_sync_edge_count =
         (uint16_t)s_startup_sync_edge_count;
     g_foc_observer_recovery_sync_active = 0U;
     g_foc_observer_sync_step_mrad = 0;
     g_foc_observer_sync_diff_mrad = 0;
     g_foc_observer_startup_pre_edge_clamp_active = 0U;
     g_foc_observer_startup_pre_edge_clamp_step_mrad = 0;
     g_foc_observer_startup_pre_edge_clamp_diff_mrad = 0;
     g_foc_observer_no_edge_angle_clamp_active = 0U;
     g_foc_observer_no_edge_angle_diff_mrad = 0;
     g_foc_observer_no_edge_angle_step_mrad = 0;

     if ((ctx->speed_ref <= 0.5f) || (cur_sector == 0U)) {
         s_startup_sync_edge_count = 0U;
         g_foc_observer_startup_sync_edge_count = 0U;
     }

     if (s_predict_direction != hall_dir) {
         s_predict_direction = hall_dir;
         s_startup_predict_speed_rpm = 0.0f;
         s_startup_sync_edge_count = 0U;
         if (cur_sector != 0U) {
             ctx->theta_e_predicted = ctx->hall_sector.theta_e;
             ctx->theta_e_prev = ctx->theta_e_predicted;
         }
         g_foc_observer_direction_reset_count++;
     }
     if (no_edge_overdue != 0U) {
         if (speed_for_predict > no_edge_limit_rpm) {
             speed_for_predict = no_edge_limit_rpm;
         }
         if (s_startup_predict_speed_rpm > no_edge_limit_rpm) {
             s_startup_predict_speed_rpm = no_edge_limit_rpm;
         }
         g_foc_observer_no_edge_elapsed_us = no_edge_elapsed_us;
         g_foc_observer_no_edge_speed_limit_rpm =
             (uint16_t)((no_edge_limit_rpm > 65535.0f)
                      ? 65535U
                      : no_edge_limit_rpm);
     }
     /* Always extrapolate by the real control interval first. */
     if ((no_edge_overdue == 0U) &&
         (use_startup_ref_predict != 0U) &&
         (ctx->speed_ref > 0.0f) &&
         (speed_for_predict < FOC_STARTUP_PREDICT_MAX_RPM)) {
         float startup_target = ctx->speed_ref_ctrl;
         float ramp_step = FOC_STARTUP_PREDICT_RAMP_RPM_PER_S * dt;

         if (startup_target < FOC_STARTUP_PREDICT_START_RPM) {
             startup_target = FOC_STARTUP_PREDICT_START_RPM;
         }
         if (startup_target > ctx->speed_ref) {
             startup_target = ctx->speed_ref;
         }
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
         g_foc_observer_startup_ref_active = 1U;
     } else {
         float release_target = speed_for_predict;
         float release_delta = release_target - s_startup_predict_speed_rpm;
         float release_step = release_blend_rate_rpm_s * dt;

         if ((ctx->speed_ref > 0.0f) &&
             (cur_sector != 0U) &&
             (s_startup_predict_speed_rpm > 0.0f) &&
             (release_step > 0.0f) &&
             (FOC_FABS(release_delta) > release_blend_done_rpm)) {
             if (release_delta > release_step) {
                 s_startup_predict_speed_rpm += release_step;
             } else if (release_delta < -release_step) {
                 s_startup_predict_speed_rpm -= release_step;
             } else {
                 s_startup_predict_speed_rpm = release_target;
             }
             speed_for_predict = s_startup_predict_speed_rpm;
             startup_release_blend_active = 1U;
         } else {
             s_startup_predict_speed_rpm = speed_for_predict;
         }
     }
     if (startup_release_blend_active != 0U) {
         g_foc_observer_startup_ref_active = 1U;
     }
     g_foc_observer_predict_speed_rpm =
         (uint16_t)((speed_for_predict > 65535.0f) ? 65535U : speed_for_predict);

     if (speed_for_predict > 0.0f) {
         omega_e = speed_for_predict * (FOC_2PI / 60.0f) * (float)pole_pairs;
         if (hall_dir == FOC_DIR_CCW) {
             ctx->theta_e_predicted -= omega_e * dt;
         } else {
             ctx->theta_e_predicted += omega_e * dt;
         }
     }

 

     /* 扇区跳变时：软同步到跳变后的扇区中心角度，

      * 避免硬同步导致角度突变引发电流尖峰 */

     FOC_Observer_ClampStartupPreEdgeAngle(ctx, cur_sector, hall_dir);

#if FOC_HALL_NO_EDGE_ANGLE_CLAMP_ENABLE
     if ((no_edge_overdue != 0U) && (cur_sector != 0U)) {
         float limit = FOC_HALL_NO_EDGE_ANGLE_LIMIT_RAD;
         float diff = FOC_Observer_NormalizeAngleDiff(ctx->theta_e_predicted -
                                                      ctx->hall_sector.theta_e);
         float step_max = FOC_HALL_NO_EDGE_ANGLE_CLAMP_STEP_RAD;
         float hard_step_max = FOC_HALL_NO_EDGE_ANGLE_CLAMP_STEP_MAX_RAD;
         float target = ctx->theta_e_predicted;
         float step;

         if (limit < 0.0f) {
             limit = -limit;
         }
         if (limit > (FOC_PI / 3.0f)) {
             limit = FOC_PI / 3.0f;
         }
         if (step_max < 0.0f) {
             step_max = -step_max;
         }
         if (hard_step_max < 0.0f) {
             hard_step_max = -hard_step_max;
         }
         if ((hard_step_max > 0.0f) && (step_max > hard_step_max)) {
             step_max = hard_step_max;
         }

         g_foc_observer_no_edge_angle_diff_mrad =
             (int16_t)(diff * 1000.0f);
         g_foc_observer_no_edge_angle_limit_mrad =
             (int16_t)(limit * 1000.0f);

         if (diff > limit) {
             target = FOC_NormalizeAngle(ctx->hall_sector.theta_e + limit);
             step_max += (diff - limit) *
                         FOC_HALL_NO_EDGE_ANGLE_CLAMP_EXTRA_FACTOR;
         } else if (diff < -limit) {
             target = FOC_NormalizeAngle(ctx->hall_sector.theta_e - limit);
             step_max += (-limit - diff) *
                         FOC_HALL_NO_EDGE_ANGLE_CLAMP_EXTRA_FACTOR;
         }
         if ((hard_step_max > 0.0f) && (step_max > hard_step_max)) {
             step_max = hard_step_max;
         }

         step = FOC_Observer_NormalizeAngleDiff(target -
                                                ctx->theta_e_predicted);
         if (step != 0.0f) {
             if ((step_max > 0.0f) && (step > step_max)) {
                 step = step_max;
             } else if ((step_max > 0.0f) && (step < -step_max)) {
                 step = -step_max;
             }

             ctx->theta_e_predicted =
                 FOC_NormalizeAngle(ctx->theta_e_predicted + step);
             g_foc_observer_no_edge_angle_step_mrad =
                 (int16_t)(step * 1000.0f);
             g_foc_observer_no_edge_angle_clamp_active = 1U;
             g_foc_observer_no_edge_angle_clamp_count++;
         }
     }
#endif

     if (cur_sector != ctx->hall_sector_prev && cur_sector != 0U) {

         if (FOC_Observer_HallStepMatchesDirection(ctx,
                                                   ctx->hall_sector_prev,
                                                   cur_sector) != 0U) {

         /* Skip sync while inertia carries the rotor opposite to the command. */
         float target = ctx->hall_sector.theta_e;
         float sync_factor = FOC_ANGLE_SYNC_FACTOR;
         float sync_step_max = FOC_ANGLE_SYNC_STEP_MAX_RAD;
         float speed_err = FOC_FABS(ctx->speed_ref - ctx->speed_filtered);
         float diff_abs;
         float sync_step;
         float recovery_step_max = FOC_ANGLE_SYNC_RECOVERY_STEP_MAX_RAD;
         uint8_t startup_sync_boost = 0U;

#if FOC_HALL_EDGE_SYNC_ENABLE
         float edge_advance_max = FOC_HALL_EDGE_SYNC_ADVANCE_MAX_RAD;

         if (g_foc_observer_startup_ref_active != 0U) {
             edge_advance_max = FOC_STARTUP_EDGE_SYNC_ADVANCE_MAX_RAD;
         }
         target = FOC_Observer_GetHallEdgeSyncAngle(ctx,
                                                    cur_sector,
                                                    omega_e,
                                                    edge_advance_max);
         sync_factor = FOC_HALL_EDGE_SYNC_FACTOR;
         sync_step_max = FOC_HALL_EDGE_SYNC_STEP_MAX_RAD;
         recovery_step_max = FOC_HALL_EDGE_SYNC_RECOVERY_STEP_MAX_RAD;
#endif

         float diff = target - ctx->theta_e_predicted;

         /* 处理角度环绕 */

         if (diff > FOC_PI)  diff -= FOC_2PI;

         if (diff < -FOC_PI) diff += FOC_2PI;

         diff_abs = FOC_FABS(diff);

         if (dt > ((float)FOC_CONTROL_LATE_PERIOD_US * 1.0e-6f)) {
             sync_factor = FOC_LATE_PERIOD_ANGLE_SYNC_FACTOR;
             sync_step_max = FOC_LATE_PERIOD_ANGLE_SYNC_STEP_MAX_RAD;
         }

         if (g_foc_observer_startup_ref_active != 0U) {
             if (s_startup_sync_edge_count <
                 (uint8_t)FOC_STARTUP_EDGE_SYNC_BOOST_COUNT) {
                 startup_sync_boost = 1U;
                 sync_factor = FOC_STARTUP_EDGE_SYNC_BOOST_FACTOR;
                 sync_step_max = FOC_STARTUP_EDGE_SYNC_BOOST_STEP_MAX_RAD;
             } else {
                 sync_factor = FOC_STARTUP_EDGE_SYNC_FACTOR;
                 sync_step_max = FOC_STARTUP_EDGE_SYNC_STEP_MAX_RAD;
             }
             g_foc_observer_startup_sync_active = 1U;
             g_foc_observer_startup_sync_boost_active = startup_sync_boost;
         } else if ((speed_err > FOC_ANGLE_SYNC_RECOVERY_SPEED_ERROR_RPM) ||
             (diff_abs > FOC_ANGLE_SYNC_RECOVERY_DIFF_RAD)) {
             g_foc_observer_recovery_sync_active = 1U;
             if (sync_factor < FOC_ANGLE_SYNC_RECOVERY_FACTOR) {
                 sync_factor = FOC_ANGLE_SYNC_RECOVERY_FACTOR;
             }
             if ((sync_step_max > 0.0f) &&
                 ((recovery_step_max <= 0.0f) ||
                  (sync_step_max < recovery_step_max))) {
                 sync_step_max = recovery_step_max;
             }
         }

         if (diff_abs > FOC_ANGLE_SYNC_RESYNC_DIFF_RAD) {
             sync_factor = 1.0f;
             sync_step_max = FOC_ANGLE_SYNC_RESYNC_STEP_MAX_RAD;
             s_startup_predict_speed_rpm = speed_for_predict;
             g_foc_observer_resync_count++;
             g_foc_observer_resync_diff_mrad = (int16_t)(diff * 1000.0f);
         }
         if (sync_factor > 1.0f) {
             sync_factor = 1.0f;
         } else if (sync_factor < 0.0f) {
             sync_factor = 0.0f;
         }

         sync_step = diff * sync_factor;

         if (sync_step_max > 0.0f) {
             if (sync_step > sync_step_max) {
                 sync_step = sync_step_max;
             } else if (sync_step < -sync_step_max) {
                 sync_step = -sync_step_max;
             }
         }

         ctx->theta_e_predicted += sync_step;
         g_foc_observer_sync_step_mrad = (int16_t)(sync_step * 1000.0f);
         g_foc_observer_sync_diff_mrad = (int16_t)(diff * 1000.0f);
         if (g_foc_observer_startup_ref_active != 0U) {
             if (s_startup_sync_edge_count < 255U) {
                 s_startup_sync_edge_count++;
             }
             g_foc_observer_startup_sync_edge_count =
                 (uint16_t)s_startup_sync_edge_count;
         }

         }

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

 

 


