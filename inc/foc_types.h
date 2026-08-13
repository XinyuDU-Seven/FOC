
/**

 * @file foc_types.h

 * @brief FOC 电机控制模块 — 通用数据类型定义

 *

 * 本文件定义 FOC 模块内部所有共用数据结构、枚举与常量。

 * 不包含硬件相关定义，保持平台无关。

 */

 

 #ifndef FOC_TYPES_H

 #define FOC_TYPES_H

 

 #include <stdint.h>

 

 #ifdef __cplusplus

 extern "C" {

 #endif

 

 /* ===================================================================

  *  枚举定义

  * =================================================================== */

 

 /** 位置传感器类型 */

 typedef enum {

     FOC_SENSOR_HALL = 0,  /**< 霍尔传感器（三路数字开关电平） */

 } FOC_SensorType_e;

 

 /** 霍尔传感器原始数据（三路数字开关电平） */

 typedef struct {

     uint8_t h1;  /**< 霍尔U相电平 (0 或 1) */

     uint8_t h2;  /**< 霍尔V相电平 (0 或 1) */

     uint8_t h3;  /**< 霍尔W相电平 (0 或 1) */

 } FOC_HallRaw_t;

 

 /** 霍尔扇区号（1~6）与角度 */

 typedef struct {

     uint8_t sector;       /**< 当前扇区号 1~6，0表示无效 */

     float   theta_e;      /**< 扇区中心电角度 (rad)，由查找表映射 */

 } FOC_HallSector_t;

 

 /** FOC 模块运行状态 */

 typedef enum {

     FOC_STATE_INIT    = 0,  /**< 初始化态：PID复位，等待配置 */

     FOC_STATE_IDLE    = 1,  /**< 待机态：PWM关闭，等待启动指令 */

     FOC_STATE_RUNNING = 2,  /**< 运行态：主循环周期执行 */

     FOC_STATE_FAULT   = 3,  /**< 故障态：PWM关闭，等待故障清除 */

 } FOC_State_e;

 

 /** 故障码（位域，可同时存在多种故障） */

 typedef enum {

     FOC_FAULT_NONE        = 0x00,  /**< 无故障 */

     FOC_FAULT_OVERCURRENT = 0x01,  /**< 过流 */

     FOC_FAULT_OVERVOLTAGE = 0x02,  /**< 过压 */

     FOC_FAULT_UNDERVOLTAGE= 0x04,  /**< 欠压 */

     FOC_FAULT_STALL       = 0x08,  /**< 堵转 */

     FOC_FAULT_HALL        = 0x10,  /**< 霍尔传感器异常 */
     FOC_FAULT_SPEED_DROP  = 0x20,  /**< Speed feedback drop diagnostic */

 } FOC_Fault_e;

 

 /** 电机旋转方向 */

 typedef enum {

     FOC_DIR_CW  = 0,  /**< 正转 */

     FOC_DIR_CCW = 1,  /**< 反转 */

 } FOC_Dir_e;

 

 /* ===================================================================

  *  坐标系数据结构

  * =================================================================== */

 

 /** 三相静止坐标系 (abc) */

 typedef struct {

     float ia;  /**< A相电流 (A) */

     float ib;  /**< B相电流 (A) */

     float ic;  /**< C相电流 (A) */

 } FOC_PhaseCurrent_t;

 

 /** 三相电流 ADC 原始采样值（驱动层直接返回，FOC port层负责解析） */

 typedef struct {

     int32_t ia_raw;  /**< A相 ADC 原始值 */

     int32_t ib_raw;  /**< B相 ADC 原始值 */

     int32_t ic_raw;  /**< C相 ADC 原始值 */

 } FOC_PhaseCurrentRaw_t;

 

 /** 电流采样标定参数（offset上电自标定，scale初始化时从HAL获取并缓存） */

 typedef struct {

     float   ia_offset;   /**< A相 ADC 零点偏移（上电自标定获取） */

     float   ib_offset;   /**< B相 ADC 零点偏移 */

     float   ic_offset;   /**< C相 ADC 零点偏移 */

     float   i_scale;     /**< 电流 ADC → 实际电流比例系数 (A/LSB)，初始化时缓存 */

     float   v_scale;     /**< 母线电压 ADC → 实际电压比例系数 (V/LSB)，初始化时缓存 */

 } FOC_CurrentCalib_t;

 

 /** 两相静止坐标系 (αβ) */

 typedef struct {

     float alpha;  /**< α轴分量 */

     float beta;   /**< β轴分量 */

 } FOC_AlphaBeta_t;

 

 /** 两相旋转坐标系 (dq) */

 typedef struct {

     float d;  /**< d轴分量（磁通方向） */

     float q;  /**< q轴分量（转矩方向） */

 } FOC_DQ_t;

 

 /* ===================================================================

  *  PID 控制器

  * =================================================================== */

 

 /** PID 运行时状态（含积分累积等内部变量，不应由外部直接修改） */

 typedef struct {

     float kp;          /**< 比例增益 */

     float ki;          /**< 积分增益 */

     float kd;          /**< 微分增益 */

     float integral;    /**< 积分累积量 */

     float prev_error;  /**< 上一次误差（用于微分计算） */

     float out_min;     /**< 输出下限 */

     float out_max;     /**< 输出上限 */

 } FOC_PID_t;

 

 /** PID 参数配置（仅参数，不含运行时状态，用于应用层传参） */

 typedef struct {

     float kp;       /**< 比例增益 */

     float ki;       /**< 积分增益 */

     float kd;       /**< 微分增益 */

     float out_min;  /**< 输出下限 */

     float out_max;  /**< 输出上限 */

 } FOC_PID_Params_t;

 

 /* ===================================================================

  *  电机参数

  * =================================================================== */

 

 /** PMSM 电机电气参数 */

 typedef struct {

     uint8_t pole_pairs;    /**< 极对数 */

     float   rs;            /**< 定子电阻 (Ω) */

     float   ls_d;          /**< d轴电感 (H) */

     float   ls_q;          /**< q轴电感 (H) */

     float   flux_linkage;  /**< 永磁体磁链 (Wb) */

     float   v_bus;         /**< 母线电压 (V) */

     float   max_speed_rpm; /**< 最大转速限制 (rpm) */

     float   max_current_a; /**< 最大电流限制 (A) */

 } FOC_MotorParams_t;

 

 /* ===================================================================

  *  模块配置

  * =================================================================== */

 

 /** FOC 模块完整配置（初始化时传入） */

 typedef struct {

     FOC_MotorParams_t    motor;         /**< 电机参数 */

     FOC_PID_Params_t     speed_pid;     /**< 速度环PID参数 */

     FOC_PID_Params_t     current_d_pid; /**< d轴电流环PID参数 */

     FOC_PID_Params_t     current_q_pid; /**< q轴电流环PID参数 */

     FOC_CurrentCalib_t   current_calib; /**< 电流采样标定参数（offset + scale缓存） */

 } FOC_Config_t;

 

 /* ===================================================================

  *  FOC 运行上下文

  * =================================================================== */

 

 /** FOC 完整运行上下文（模块内部使用，外部只读） */

 typedef struct {

     /* ---- 原始传感器数据（驱动层直接返回） ---- */

     FOC_HallRaw_t          hall_raw;         /**< 霍尔传感器三路原始电平 */

     FOC_PhaseCurrentRaw_t  i_abc_raw;        /**< 三相电流 ADC 原始值 */

     uint32_t               v_bus_raw;        /**< 母线电压 ADC 原始值 */

 

     /* ---- 解析后的物理量（由 observer/port 层从原始数据换算） ---- */

     float                  theta_m;          /**< 机械角度 (rad) [0, 2π) */

     float                  speed_ref;        /**< 目标转速 (rpm) */

     float                  speed_ref_ctrl;   /**< Slew-limited speed reference for control/protection (rpm) */

     float                  speed_fdb;        /**< Filtered speed feedback (rpm) */

     float                  theta_e;          /**< 电角度 (rad) */

     FOC_PhaseCurrent_t     i_abc;            /**< 三相实际电流 (A) */

     float                  v_bus;            /**< 母线电压 (V) */

 

     /* ---- 坐标变换中间量 ---- */

     FOC_AlphaBeta_t        i_ab;             /**< Clarke 变换输出 (αβ电流) */

     FOC_DQ_t               i_dq;             /**< Park 变换输出 (dq电流反馈) */

     FOC_DQ_t               v_dq;             /**< PID 输出 (dq电压) */

     FOC_AlphaBeta_t        v_ab;             /**< 逆Park 变换输出 (αβ电压) */

 

     /* ---- PID 控制器实例 ---- */

     FOC_PID_t              pid_speed;        /**< 速度环 PID */

     FOC_PID_t              pid_id;           /**< d轴电流环 PID */

     FOC_PID_t              pid_iq;           /**< q轴电流环 PID */

 

     /* ---- 电流环参考 ---- */

     float                  id_ref;           /**< d轴电流参考（通常为0） */

     float                  iq_ref;           /**< q轴电流参考（速度环输出） */

 

     /* ---- SVPWM 输出 ---- */

     float                  duty_a;           /**< A相占空比 [0,1] */

     float                  duty_b;           /**< B相占空比 [0,1] */

     float                  duty_c;           /**< C相占空比 [0,1] */

 

     /* ---- 状态 ---- */

     FOC_State_e            state;            /**< 当前运行状态 */

     FOC_Fault_e            fault;            /**< 当前故障码 */

     FOC_Dir_e              direction;        /**< 旋转方向 */

 

     /* ---- 观测器内部状态 ---- */

     FOC_HallSector_t       hall_sector;      /**< 当前霍尔扇区与电角度 */

     uint8_t                hall_sector_prev;  /**< 上一次霍尔扇区号 */

     float                  theta_e_prev;     /**< 上一次电角度 (rad) */

     float                  speed_raw;        /**< Signed raw Hall speed estimate (rpm) */

     float                  speed_filtered;   /**< 滤波后转速 (rpm) */

     float                  speed_ctrl_fdb;   /**< Speed feedback used by speed PID (rpm) */

     uint32_t               timestamp_prev;   /**< 上一次扇区跳变时间戳 (us) */

     uint32_t               hall_sector_dt_us; /**< 上一次有效扇区间隔 (us) */

     uint16_t               sector_no_change_count; /**< 连续无扇区跳变的控制周期计数 */

     float                  theta_e_predicted; /**< 扇区间插值预测电角度 (rad) */

 

     /* ---- 保护检测状态 ---- */

     float                  current_peak;     /**< 峰值电流 (A) */

     uint32_t               stall_counter;    /**< 堵转计数 */

 

     /* ---- 速度环降采样计数 ---- */

     uint16_t               speed_loop_counter; /**< 速度环降采样计数器 */

     uint32_t               hall_sector_timestamp_us; /**< Hall edge timestamp for current sector (us) */

 } FOC_Context_t;

 

 #ifdef __cplusplus

 }

 #endif

 

 #endif /* FOC_TYPES_H */

 

 


