
/**

 * @file FOC_DataType.h

 * @author FocTeam (FocTeam@byd.com)

 * @brief 数据类型定义

 * @version 1.0.0

 * @date 2025-09-12

 *

 * @copyright Copyright (c) 2025

 *

 */

 

#ifndef _FOC_DATA_TYPE_H_

#define _FOC_DATA_TYPE_H_

#include <stdbool.h>

#include <stdint.h>

 

/* 电机FOC占空比输出 */

typedef struct T_FocSvpwmDuty

{

    uint32_t unIsUpdated; /*本次结果是否更新，0：否*/

    uint32_t unDutyA;     /* A相占空比, 0~10000表示0%-100% */

    uint32_t unDutyB;     /* B相占空比, 0~10000表示0%-100% */

    uint32_t unDutyC;     /* C相占空比, 0~10000表示0%-100% */

} FocSvpwmDuty;

 

/*FOC算法错误码定义 */

typedef enum

{

    FOC_SUCCESS = 0x000u,                  /* 成功 */

    FOC_MOTOR_ID_INVALID = 0x0001u,        /* 电机ID不合法 */

    FOC_INPUT_PARAMETER_INVALID,           /* 输入参数异常，通用型 */

    FOC_MOTOR_DISABLED,                    /* 电机未使能 */

    FOC_MOTOR_STUCKED_BY_HALL,             /* 电机堵转 */

    FOC_MOTOR_READ_CURRENT_FAILED,         /*  ADC电流获取失败 */

    FOC_VOLTAGE_SETTING_RANGE_INVALID,     /*  电压设定数值超出范围 */

    FOC_UDC_LESS_OR_EQUALS_ZEROS,          /* 电机的Udc小于等于0 */

    FOC_UAUB_LEAD_TO_WRONG_SECTOR,         /* Ualpha, Ubeta组合计算得的扇区编号超出正常范围 */

    FOC_HALL_GET_STATES_FAILED,            /* 读取霍尔状态失败; */

    FOC_HALL_STATES_INVALID,               /* 霍尔状态无效; */

    FOC_HALL_ESTIMATE_FAILED,              /* 霍尔估计失败无效; */

    FOC_HALL_TIME_NOT_MATCH,               /* 霍尔更新时间比FOC更新时间超前; */

    FOC_HALL_HISTORY_TIME_NOT_INCREASING,  /* 霍尔历史时间非单调递增; */

    FOC_HALL_SECTOR_NOT_CHANGE,            /* 霍尔回调被触发，但是扇区未发生变化 */

    FOC_HALL_SECTOR_CHANGE_INVALID,        /* 扇区编号变化不正常 */

    FOC_HALL_CALIBRATE_TIME_LOWER_EXCEEED, /* 电角度偏差标定时间小于0 */

    FOC_HALL_CALIBRATE_TIME_UPPER_EXCEEED, /* 电角度偏差标定时间大于10 */

    FOC_HALL_CALIBRATE_INTERVAL_INVALID,   /* theta 增量角为负数或者超过theta设定范围 */

    FOC_HALL_CALIBRATE_MODE_INVALID,       /* 校正模式不合法 */

 

    FOC_HALL_CALIBRATE_RANGE_INVALID, /* [fThetaMin,fThetaMax]均应为正数，且应满足fThetaMin<fThetaMax且fThetaMax-fThetaMin

                                       */

                                      /* <= 2PI,且fThetaMax<= 2PI */

    FOC_HALL_CALIBRATE_FAILED,            /* 电角度校正失败 */

    FOC_CURRENT_CALIBRATE_COUNTS_INVALID, /* 电流校正计数不合法 */

    FOC_ALIGN_VOLTAGE_INVALID,            /* 转子对齐电压数值不合法 */

 

    FOC_PWM_SET_VALUE_SUCCESS,

    FOC_PWM_SET_VALUE_FAILED,

    FOC_PWM_SET_ZERO_FAILED,

    FOC_PWM_SET_ZERO_SUCCESS,

    FOC_PHASE_A_CURRENT_EXCEED,               /* A相电流超限 */

    FOC_PHASE_B_CURRENT_EXCEED,               /* B相电流超限 */

    FOC_PHASE_C_CURRENT_EXCEED,               /* C相电流超限 */

    FOC_PHASE_D_CURRENT_EXCEED,               /* Id电流超限 */

    FOC_PHASE_Q_CURRENT_EXCEED,               /* Iq电流超限 */

    FOC_PHASE_VOLTAGE_HIGH_EXCEED,            /* 高压超限 */

    FOC_PHASE_VOLTAGE_LOW_EXCEED,             /* 低压超限 */

    FOC_PHASE_DQ_SQRT_CURRENT_EXCEED,         /* Iq Id均方根相连续越线超限 */

    FOC_PHASE_ABC_SQRT_CURRENT_EXCEED,        /*  IABC均方根相连续越线超限 */

    FOC_MOTOR_STUCKED,                        /* 电机堵转 */

    FOC_MODIFYING_PARAMETER_WHILE_RUNNING,    /* 电机运行中，不能修改参数 */

    FOC_TOO_LARGE_SECTOR_SIZE,                /* 霍尔扇区角度偏移值设定时，数组大小不为6 */

    FOC_EXIT_BY_ARRAY_SIZE_REACH,             /* buffer尺寸写满 */

    FOC_CURRENT_LOOPS_ELAPLSE_TIME_TOO_SMALL, /* 时间过小 */

    FOC_RAMP_AXIS_CURRENT_D_FAILED,           /* D轴电流斜坡控制失败 */

    FOC_RAMP_AXIS_CURRENT_Q_FAILED,           /* Q轴电流斜坡控失败 */

    FOC_RAMP_AXIS_VOLTAGE_D_FAILED,           /* D轴电压斜坡控失败 */

    FOC_RAMP_AXIS_VOLTAGE_Q_FAILED,           /* Q轴电压斜坡控失败 */

    FOC_POINTER_NULL,                         /* 空指针 */

    FOC_INVALID_INDEX,                        /* 无效索引 */

    FOC_SPEED_LOOP_NOT_CALC,                  /* 速度环没有进行计算操作 */

    FOC_INVALID_DIRECITON,                    /* 方向数值无效 */

    FOC_COSF_RESULT_ERROR,                    /* 调用cosf返回异常 */

    FOC_SINF_RESULT_ERROR,                    /* 调用sinf返回异常 */

    FOC_INVALID_MOTOR_POLEPAIRES,             /* 无效的电机极对数 */

    FOC_INVALID_MOTOR_ABZ_PPR,                /* 无效的电机ABZ每转脉冲数 */

    FOC_INVALID_MOTOR_ABZ_DECODE_MODE,        /* 无效的ABZ编码模式 */

    FOC_INVALID_MOTOR_ABZ_CPR,                /* 无效的电机ABZ每转计数 */

    FOC_ABZ_TIME_NOT_MATCH,                   /* ABZ更新时间比FOC更新时间超前 */

    FOC_ABZ_Z_COUNTER_NOT_MATCH,              /* ABZ编码器触发Z信号，但读数没发生跳变 */

    FOC_ABNORMAL_INTERVAL_TIME,               /* 两次时间间隔异常 */

    FOC_INVALID_ZERO_CURRENT_COUNT,           /* 无效偏置电流采样次数 */

} FocError;

 

/* 电机类型 */

typedef enum

{

    FOC_MOTOR_TYPE_UNKNOWN = 0x000u, /* 未知类型电机 */

    FOC_MOTOR_TYPE_HALL,             /* 三霍尔电机 */

    FOC_MOTOR_TYPE_ABZ,              /* ABZ磁编码器电机 */

    FOC_MOTOR_TYPE_NO_SENSOR,        /* 无传感器电机 */

    FOC_MOTOR_TYPE_SMC_HALL,         /* 霍尔+SMC */

} FocMotorType;

 

typedef enum

{

    FOC_CTRL_MODE_STOP,                 /* 停止模式 */

    FOC_CTRL_MODE_CURRENT_RECURSIVE,    /* 电流往复控制模式 */

    FOC_CTRL_MODE_CURRENT_CLOSE,        /* 电流闭环控制模式 */

    FOC_CTRL_MODE_PROPORTIONAL_CURRENT, /* 电压比例+电流 */

    FOC_CTRL_MODE_SPEED_CLOSE,          /* 速度控制模式 */

    FOC_CTRL_MODE_PULLUP_LOOP,          /* 持续强拖控制模式 */

    FOC_CTRL_MODE_SIX_STEP,             /* 六步换相法模式 */

    FOC_CTRL_MODE_VF,                   /* 持续强拖V/F模式 */

    FOC_CTRL_MODE_VF_WITH_CURRENT,      /* 持续强拖V/F模式，根据电流设置的强拖电压 */

    FOC_CTRL_MODE_VOLTAGE_CLOSE,        /* 指定Vd，Vq的闭环模式 */

    FOC_CTRL_MODE_PROPORTIONAL_SPEED,   /* 电压比例+速度 */

    FOC_CTRL_MODE_IF,                   /* 持续强拖I/F模式 */

    FOC_CTRL_MODE_CURRENT_ZERO_ALIGN,   /* 电流校正模式 */

    FOC_CTRL_MODE_ROTOR_ALIGN,          /* 转子对齐模式 */

    FOC_CTRL_MODE_TORQUE_CLOSE,         /* 扭矩闭环模式 */

    FOC_CTRL_MODE_SMC_SPEED_CLOSE,      /*带SMC的速度闭环控制模式*/

    FOC_CTRL_MODE_SMC_SPEED_FF,      /*带前馈的SMC的速度闭环控制模式*/

} FocControlMode;

 

typedef struct

{

    uint16_t unMotorId;        /* 电机ID，0：水平电机，1：高调电机 */

    uint16_t unRunState;       /* 电机状态控制，0：停机，1：运行 */

    FocControlMode enCtrlMode; /* 电机控制模式， */

    uint16_t

        unSpeedEstimatePhase; /* 角度估计阶段，0：初始化，1：开环强拖指定角度，2：闭环估算角度, 3: 持续强拖阶段，4: */

                              /* 六步换向阶段，5： VF阶段,6: 停止中，7：I/F阶段 */

    uint16_t unDirection;     /* 控制方向, 0: 无方向，1: 正方向，2: 反方向 */

    int16_t nSpeedRefRpm;     /* 速度环速度目标，RPM，原始值 */

    int32_t nIdRefmA;         /* 电流环id目标，mA，原始值 */

    int32_t nIqRefmA;         /* 电流环iq目标，mA，原始值 */

    int32_t nVdRefmV;         /* 电流环Vd目标，mV，原始值 */

    int32_t nVqRefmV;         /* 电流环Vq目标，mV，原始值 */

    int32_t nTorqueRefmNm;    /* 参考扭矩，mN·m，原始值  */

 

    int16_t nSpeedRef;  /* 速度环速度目标，RPM，标幺值 */

    int16_t nIdRef;     /* 电流环id目标，mA，标幺值 */

    int16_t nIqRef;     /* 电流环iq目标，mA，标幺值 */

    int16_t nVdRef;     /* 电流环Vd目标，mV，标幺值 */

    int16_t nVqRef;     /* 电流环Vq目标，mV，标幺值 */

    int16_t nTorqueRef; /* 参考扭矩，mN·m，标幺值  */

 

    /* 以下待删除 */

    float fSpeedRef;     /* 速度环速度目标，RPM */

    float fSpeedRefExec; /* 速度环执行速度，RPM */

    float fIdRef;        /* 电流环d轴目标电流 */

    float fIqRef;        /* 电流环q轴目标电流 */

    float fIdRefExec;    /* 电流环d轴执行电流 */

    float fIqRefExec;    /* 电流环q轴执行电流 */

    float fVqRef;        /* Q轴参考电压，V */

    float fVdRef;        /* D轴参考电压，V */

    /* 以上待删除 */

 

    uint16_t unFactor; /* 比例系数，如电压比例控制时，输入电压所占比例，0~32727表示0%-100% */

    uint16_t unFactorPercent; /* 比例系数，0-10000, 表示0%-100% */

 

    float fIncreamentTheta;     /* 强拖阶段时的每个载波周期theta角增量，弧度Rad */

    uint32_t unCalibrateCounts; /* 电流校正时的次数限制 */

    float fThetaElecAlignDeg;   /* 对齐操作时的目标角度，° */

} MotorTargetCmd;

 

typedef struct

{

    uint16_t unMotorId;   /* 电机ID，0：水平电机，1：高调电机 */

    uint64_t unStartTime; /* 启动时间 */

    uint64_t unStopTime;  /* 预期结束时间 */

    int16_t nDirectionId; /* Id控制方向, -1：递减，1：递增 */

    int16_t nDirectionIq; /* Id控制方向, -1：递减，1：递增 */

    float fIdMax;         /* 电流环d轴目标最大电流 */

    float fIdMin;         /* 电流环d轴目标最小电流 */

    float fIdInterval;    /* 电流环d轴目标电流间隔 */

    float fIqMax;         /* 电流环q轴目标最大电流 */

    float fIqMin;         /* 电流环q轴目标最小电流 */

    float fIqInterval;    /* 电流环q轴目标电流间隔 */

    float fIdExec;        /* 电流环d轴执行电流 */

    float fIqExec;        /* 电流环q轴执行电流 */

} CurrentRecursiveLoopTarget;

 

typedef struct

{

    uint16_t unMotorId; /* 电机ID*/

    int32_t nCurrentLimitation;

    uint32_t unDangerCycles;       /* 连续超越安全电流FOC周期数限制 */

    uint32_t unPhaseAExceedCycles; /* A相连续越线次数 */

    uint32_t unPhaseBExceedCycles; /* B相连续越线次数 */

    uint32_t unPhaseCExceedCycles; /* C相连续越线次数 */

    uint32_t unPhaseDExceedCycles; /* Id相连续越线次数 */

    uint32_t unPhaseQExceedCycles; /* Iq相连续越线次数 */

 

} FocCurrentSafetyState;

 

typedef struct

{

    uint16_t unMotorId; /* 电机ID*/

    int16_t nVolatageHighLimitation;

    int16_t nVolatageLowLimitation;

    uint32_t unDangerCycles;            /* 连续超越安全电流FOC周期数限制 */

    uint32_t unHighVoltageExceedCycles; /* 低压连续越线次数 */

    uint32_t unLowVoltageExceedCycles;  /* 高压连续越线次数 */

} FocVoltageSafetyState;

 

typedef struct

{

    uint16_t unMotorId;       /* 电机ID，0：水平电机，1：高调电机 */

    uint8_t unIsSlowCtrl;     /* 是否缓起缓停 */

    float fCurrentLimitation; /* 电流限制 */

    float fIncreamentCurrent; /* 电流增量 */

    float fVotageLimitation;  /* 电压限制 */

    float fIncreamentVoltage; /* 电流增量 */

 

    float fIdExec;          /* d轴执行电流 */

    float fIdRef;           /* d轴目标电流 */

    float fIqExec;          /* q轴执行电流 */

    float fIqRef;           /* q轴目标电流 */

    bool bHasReachIdTarget; /* 标志电流是否已经完成爬坡 */

    bool bHasReachIqTarget; /* 标志电流是否已经完成爬坡 */

 

    float fVExec;          /* 执行电压 */

    float fVRef;           /* 目标电压 */

    float fSpeedRpsRef;    /* 目标速度 */

    bool bHasReachVTarget; /* 标志电流是否已经完成爬坡 */

} FocSlowControl;

 

typedef struct

{

    float fSpeedRef;         /* 速度环当前目标，rad/s */

    float fSpeedReal;        /* 速度环当前速度，rad/s */

    float fSpeedRefRpm;      /* 速度环当前目标，RPM */

    float fSpeedRealRpm;     /* 速度环当前速度，RPM */

    float fSpeedFilteredRpm; /* 速度环滤波后速度，RPM */

    int8_t nDirection;       /* 霍尔运动方向，-1：顺时针，1：逆时针 */

} RotorSpeed;

 

typedef struct

{

    float fThetaMech;         /* 转子的机械角度 */

    float fThetaElec;         /* 转子的电角度 */

    float fCosineOfTheta;     /* 转子电角度的cos值 */

    float fSineOfTheta;       /* 转子电角度的sin值 */

    float fThetaPll;          /* PLL估计角度 */

    float fThetaHallEstimate; /* Hall外插法角度 */

    float fThetaIFEstimate;   /* IF时估算角度法角度 */

} RotorAngle;

 

 

typedef struct

{

    float fIalpha; /* fIalpha */

    float fIbeta;  /* fIbeta */

} AlphaBetaCurrent;

 

typedef struct

{

    float fId;         /* I_d电流 */

    float fIq;         /* I_q电流 */

    float fIdFiltered; /* I_d滤波电流 */

    float fIqFiltered; /* I_q滤波电流 */

} DQCurrent;

 

typedef struct

{

    float fVd; /* V_d电压 */

    float fVq; /* V_q电压 */

} DQVoltage;

 

typedef struct

{

    float fValpha;   /* V_alpha电压 */

    float fVbeta;    /* V_beta电压 */

    int16_t nValpha; /* V_alpha电压 */

    int16_t nVbeta;  /* V_beta电压 */

} AlphaBetaVoltage;

 

typedef struct

{

    float fPidOut;       /* 当前时刻的PID控制量 */

    float fSum;          /* 上一时刻的积分项 */

    float fError;        /* 上一时刻的控制误差 */

    float fKp;           /* 控制参数P */

    float fKi;           /* 控制参数I */

    float fKd;           /* 控制参数D */

    float fGainB;        /* 控制参数B */

    float fHighLimit;    /* PID上限 */

    float fLowLimit;     /* PID下限 */

    int32_t nLoopCounts; /* 循环递减轮数 */

    float fRamp;         /* 输出占比递增斜率 */

    float fRatio;        /* 当前输出占比 */

    float fPidRef;       /* 开-闭切换时的参考值 */

    float fErrorLast;

    float fAntiWindup; /* 抗积分饱和 */

} PidCtl;

 

 

typedef struct

{

    float fTa;        /* A相占空比 */

    float fTb;        /* B相占空比 */

    float fTc;        /* C相占空比 */

    uint32_t unDutyA; /* A相占空比, 0~10000表示0%-100% */

    uint32_t unDutyB; /* B相占空比, 0~10000表示0%-100% */

    uint32_t unDutyC; /* C相占空比, 0~10000表示0%-100% */

} SvpwmOutput;

 

typedef struct

{

    uint8_t unId;                          /* 电机编号 */

    uint8_t unHallState;                   /* 当前霍尔状态 */

    uint64_t unTotalHallCounts;            /* 霍尔更新次数计数 */

    uint64_t unCurrentDirectionHallCounts; /* 换向后的霍尔更新次数计数 */

    int64_t

        nHallDistance; /* 霍尔连续移动距离，正值表示逆时针移动数量，负值表示顺时针移动的数量，在从系统激活的时候赋零 */

    float fSpeedMechEstimate;         /* 当前机械角速度，RPM，转每分钟 */

    float fSpeedMechEstimateFiltered; /* 当前滤波的机械角速度，RPM，转每分钟 */

    float fThetaElec;                 /* 当前电角度，弧度 */

    float fId;                        /* I_d电流 ，A */

    float fIq;                        /* I_q电流，A */

    float fIqRef;                     /* 电流环q轴目标电流，A */

    float fIa;                        /* A/U相电流，A */

    float fIb;                        /* B/V相电流，A */

    float fIc;                        /* C/W相电流，A */

    float fUd;                        /* d轴电压 ，V */

    float fUq;                        /* q轴电压，V */

    uint8_t unNumberPoles;            /* 极对数 */

    uint64_t unUpdateTime;            /* 当前霍尔更新时系统时间 */

    FocError enFocState;              /* FOC电流环运算状态 */

    /* 新增算法应用需求数据存储 */

    uint32_t punDeltaTimeUsHall[100]; /* 用于存储霍尔的更新数据时的相对时间数组，微秒 */

    uint8_t punHistoryHall[100];      /* 用于存储霍尔的状态数组，a相高位 */

    int16_t unHeadIndexHall; /* punDeltaTimeUsHall及punHistoryHall 的更新索引（该索引上的数据已被更新） */

    /* 新增应用的霍尔记录 */

    uint64_t punHallCountsHistory[100]; /* 霍尔总数计数 */

    int16_t unHeadIndexAppHall; /* punHallCountsHistory 的更新索引（该索引上的数据已被更新） */

    uint16_t unDirection;       /* 控制方向, 0: 无方向，1: 正方向，2: 反方向 */

 

} MotorFullStates;

/*! for calibrate */

typedef struct

{

    uint8_t unId;               /* 电机编号 */

    float fThetaTestingCurrent; /* 当前角度偏差 */

    float fNegativeSpeedRangeErrorCurrent;

    float fPositiveSpeedRangeErroCurrentr;

    float fSpeedErrorCurrent;   /* 速度误差 */

    float fSpeedMeanMaxCurrent; /* 速度误差 */

    float fThetaTestingBest;    /* 当前角度偏差 */

    float fSpeedErrorBest;      /* 速度误差 */

    float fPercentage;          /* 校正进展比例 */

    uint8_t unMode;             /* 模式 */

} CalibrateInfo;

 

typedef struct

{

    uint16_t unMotorId;       /* 电机ID */

    FocMotorType enMotorType; /* 电机类型 */

    uint8_t unPolePairs;      /*  极对数-水平电机 */

    float fRs;                /* 水平电机线电阻，欧姆Ω，一般在0.1~1之间 */

    float fLd;                /* 水平电机D轴电感，亨利H,一般0.001~0.006之间，表贴式Ld=Lq */

    float fLq;                /* 水平电机Q轴电感，亨利H,一般0.001~0.006之间，表贴式Ld=Lq */

    float fFlux;              /* 水平电机磁链，韦伯Wb，一般在0.01~0.1之间 */

    uint16_t nPulsePerRound;  /* 磁编码线数 */

    uint8_t unDecodeMode;     /* 解码模式：1: X1, 2: X2, 4: X4 */

    int32_t nCounterPerRound; /* 最大编码数 nPulsePerRound*unDecodeMode - 1 */

 

} MotorProperties;

 

 

/* 电机FOC控制实际目标执行值，用于保存中间状态，分离控制与执行中断的数据 */

typedef struct T_FocExecutionTarget

{

    uint16_t unExecutionPhase; /* 执行阶段，0：初始化，1：开环强拖指定角度，2：闭环估算角度, 3: 持续强拖阶段，4: */

    /* 六步换向阶段，5： VF阶段,6: 停止中，7：I/F阶段 */

    int16_t nIdExec; /* Id 执行值，标幺化值 */

    int16_t nIqExec; /* Iq 执行值，标幺化值 */

 

    int16_t nVdExec; /* Vd 执行值，标幺化值*/

    int16_t nVqExec; /* Vq 执行值，标幺化值*/

 

    int16_t nSpeedRpmExec;  /* 速度 执行值，标幺化值 */

    int16_t nTorquemNmExec; /* 扭矩 执行值，标幺化值*/

    uint16_t unFactor;      /* 混合模式时的比例，标幺化值 */

} FocExecutionTarget;

 

/* 电机定子状态 */

typedef struct T_MotorStatorState

{

    uint16_t unId; /*电机ID */

    int16_t nIa;   /* A相电流，标幺化 */

    int16_t nIb;   /* A相电流，标幺化 */

    int16_t nIc;   /* A相电流，标幺化 */

    int16_t nUdc;  /* 母线电压，标幺化 */

 

    int16_t nIalpha; /* alpha轴电流，标幺化 */

    int16_t nIbeta;  /* beta轴电流，标幺化 */

 

    int16_t nValpha; /* alpha轴电压，标幺化 */

    int16_t nVbeta;  /* beta轴电压，标幺化 */

 

    int32_t nIamA;  /* A相电流，绝对电流，mA */

    int32_t nIbmA;  /* B相电流，绝对电流，mA */

    int32_t nIcmA;  /* C相电流，绝对电流，mA */

    int32_t nUdcmV; /* 母线电压，绝对电压，mV */

    int32_t nIdcmA; /* 母线电流，绝对电流，mA */

 

    int32_t nIamAFiltered;  /* 滤波的A相电流，绝对电流，mA */

    int32_t nIbmAFiltered;  /* 滤波的B相电流，绝对电流，mA */

    int32_t nIcmAFiltered;  /* 滤波的C相电流，绝对电流，mA */

    int32_t nUdcmVFiltered; /* 滤波的母线电压，绝对电压，mV */

} MotorStatorState;

 

/* 电机转子状态 */

typedef struct T_MotorRotorState

{

    uint16_t unId; /*电机ID */

 

    int16_t nId; /* d轴电流，标幺化 */

    int16_t nIq; /* q轴电流，标幺化 */

 

    int32_t nIdmA;         /* d轴电流，绝对值 */

    int32_t nIqmA;         /* q轴电流，绝对值  */

    int32_t nIdmAFiltered; /* 滤波d轴电流，绝对值  */

    int32_t nIqmAFiltered; /* 滤波q轴电流，绝对值  */

 

    int16_t nIdFiltered; /* 滤波的d轴电流，标幺化 */

    int16_t nIqFiltered; /* 滤波的q轴电流，标幺化 */

 

    /* ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 角度估计 Foc_RotorAngleEstimateHandler应该修改的变量  ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/

    int16_t

        nThetaElecRad; /* 电角度（预测值或者计算值，将会被后续环节使用），1/10000弧度，即±31415表示±3.1415弧度[PI] */

    int16_t

        nThetaMechRad; /* 机械角度（预测值或者计算值，将会被后续环节使用），1/10000弧度，即±31415表示±3.1415弧度[PI] */

    int16_t

        nThetaElecRadCalibrate; /* 电角度校正值，1/10000弧度，即±31415表示±3.1415弧度[PI]，即霍尔或者ABZ触发时的准确值

                                 */

    int16_t nThetaErrorRadCalibrate; /* 触发校正时的角度误差，1/10000弧度，即±31415表示±3.1415弧度[PI] */

    uint64_t unThetaCalibrateTimeUs; /* 电角度校正时的系统时间，us微秒，即霍尔或者ABZ触发时的时间 */

 

    int16_t

        nThetaElecRadNormalized; /* 电角度（预测值或者计算值，将会被后续环节使用），标幺化，±INT16_MAX映射到-Pi~PI弧度

                                  */

    int16_t

        nThetaMechRadNormalized; /* 机械角度（预测值或者计算值，将会被后续环节使用），标幺化，±INT16_MAX映射到-Pi~PI弧度

                                  */

    /* ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ 角度估计 Foc_RotorAngleEstimateHandler应该修改的变量  ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

 

    int16_t nCosineOfTheta; /* 电角度余弦值，标幺化，映射到-1~1 */

    int16_t nSineOfTheta;   /* 电角度正弦值，标幺化，映射到-1~1 */

 

    /* ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 速度估计 Foc_RotorAngleEstimateHandler 应该修改的变量  ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/

    int16_t nSpeedRealRpm;   /* 转子机械角速度，转每分钟 */

    int16_t nSpeedRealRadps; /* 转子机械角速度，1/10弧度每秒 */

    int16_t nSpeedRealRpmFiltered; /* 滤波后转子机械角速度，转每分钟，无滤波时，应该赋值为nSpeedRealRpm */

    int16_t nSpeedRealRpmFilteredNormalized; /* nSpeedRealRpmFiltered的标幺化数值 */

    /* ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ 速度估计 Foc_RotorAngleEstimateHandler 应该修改的变量  ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

 

    int16_t nTorqueRealmNm; /* 当前扭矩，mNm */

 

} MotorRotorState;

 

/* 电机诊断状态 */

typedef struct T_MotorDiagnoseState

{

    uint16_t unId; /*电机ID */

 

    int32_t nIamA;  /* A相电流，绝对电流，mA */

    int32_t nIbmA;  /* B相电流，绝对电流，mA */

    int32_t nIcmA;  /* C相电流，绝对电流，mA */

    int32_t nUdcmV; /* 母线电压，绝对电压，mV */

 

    int16_t nId; /* d轴电流，标幺化 */

    int16_t nIq; /* q轴电流，标幺化 */

 

    int16_t nIdExec; /* Id 执行值，标幺化值 */

    int16_t nIqExec; /* Iq 执行值，标幺化值 */

 

    uint32_t unDutyA; /* A相占空比, 0~10000表示0%-100% */

    uint32_t unDutyB; /* B相占空比, 0~10000表示0%-100% */

    uint32_t unDutyC; /* C相占空比, 0~10000表示0%-100% */

 

    uint8_t unIsOverHighVoltage; /* 预驱是否存在过高压，0：否，1：是 */

    uint8_t unIsOverLowVoltage;  /* 预驱是否存在过低压，0：否，1：是 */

    uint8_t unIsOverCurrent;     /* 预驱是否存在过流，0：否，1：是 */

    uint8_t unIsOverTemperature; /* 预驱是否存在过高温，0：否，1：是 */

 

    int16_t nSpeedRealRpm; /* 转子实际机械角速度，转每分钟 */

    int16_t nSpeedRefRpm;  /* 转子目标机械角速度，转每分钟 */

    int8_t nDirection;     /* 目标运动方向 0: 未知/无效, 1:正转, 2: 反转 */

 

    int32_t nTorqueRefmNm; /* 参考扭矩，mN·m，原始值  */

 

    uint8_t unHallState;        /* 当前霍尔状态 */

    uint64_t unTotalHallCounts; /* 霍尔更新次数计数 */

    uint64_t unHallUpdateTime;  /* 当前霍尔更新时系统时间 */

 

    uint64_t unAbzUpdateTime;      /* 当前ABZ更新时系统时间 */

    uint8_t unIsCalibrateByZ;      /* 是否经过了实时Z信号的校正 */

    uint8_t unIsCalibrateByMemory; /* 是否已经经过读取持久化数据的校正 */

    int32_t nCurrentDecodeValue;   /* 当前解码数值 */

 

    int16_t

        nThetaElecRad; /* 电角度（预测值或者计算值，将会被后续环节使用），1/10000弧度，即±31415表示±3.1415弧度[PI] */

    int16_t nThetaElecRadCalibrate; /* 电角度校正值，1/10000弧度，即±31415表示±3.1415弧度[PI] */

 

    FocError enFocState; /* FOC驱动模块运算状态 */

 

} MotorDiagnoseState;

 

/********************************************平台化函数指针定义********************************************/

/* 电流电压采样函数指针, 结果应该存到定子结构体指针当中，参数分别为：电机Id，定子结构体指针 */

typedef FocError (*Foc_CurrentVoltageSamplingHandler)(uint8_t, MotorStatorState *, FocSvpwmDuty *);

 

/* 角度估计函数指针，转子角度应该返回到转子结构体指针中（见定义说明），参数分别为：电机Id，电机属性指针，定子结构体指针，转子结构体指针

 */

typedef FocError (*Foc_RotorAngleEstimateHandler)(uint8_t, MotorProperties *, MotorStatorState *, MotorRotorState *);

 

/* 机械角速度估计函数指针，转子角速度应该返回到转子结构体指针中（见定义说明），参数分别为：电机Id，电机属性指针，定子结构体指针，转子结构体指针

 */

typedef FocError (*Foc_RotorSpeedEstimateHandler)(uint8_t, MotorProperties *, MotorStatorState *, MotorRotorState *);

 

/* Clarke变换函数指针，参数分别为：UVW(ABC)输入，转换后Alpha、Beta结果指针 */

typedef FocError (*Foc_ClarkeHandler)(int16_t, int16_t, int16_t, int16_t *, int16_t *);

 

/* 三角函数计算函数指针，参数分别为：theta输入(1/10000弧度)，转换后nCosineOfTheta、nSineOfTheta结果指针 */

typedef FocError (*Foc_CosSinHandler)(int16_t, int16_t *, int16_t *);

 

/*

Park变换：AlphaBeta->DQ

Park变换函数指针，参数分别为：alphabeta输入，nCosineOfTheta、nSineOfTheta,转换后d轴、q轴结果指针

*/

typedef FocError (*Foc_ParkHandler)(int16_t, int16_t, int16_t, int16_t, int16_t *, int16_t *);

 

/*

反Park变换：DQ->AlphaBeta

反Park变换函数指针，参数分别为：d轴、q轴输入，nCosineOfTheta、nSineOfTheta,转换后alpha轴、beta轴结果指针

*/

typedef FocError (*Foc_InvParkHandler)(int16_t, int16_t, int16_t, int16_t, int16_t *, int16_t *);

 

/*

Id, Iq执行值计算函数指针，即速度环（扭矩环）输出

参数分别为：电机Id，扭矩/速度期望值，扭矩/速度当前值，结果为d轴、q轴目标执行电流结果指针

*/

typedef FocError (*Foc_CurrentExecutionHandler)(uint8_t, int16_t, int16_t, int16_t *, int16_t *);

 

/*

Vd, Vq执行值计算函数指针，即电流环输出

参数分别为：电机Id，d轴、q轴期望电流（标幺化），d轴、q轴当前电流（标幺化），结果为d轴、q轴目标执行电压结果指针

*/

typedef FocError (*Foc_VoltageExecutionHandler)(uint8_t, int16_t, int16_t, int16_t, int16_t, int16_t *, int16_t *);

 

/*

SVPWM计算函数指针，参数分别为：nValpha、nVbeta输入，母线电压nUdc，占空比（0~10000对应0%-100%）unDutyA、unDutyB、unDutyC的结果指针

*/

typedef FocError (*Foc_SvpwmHandler)(int16_t, int16_t, int16_t, uint32_t *, uint32_t *, uint32_t *);

 

/******************************************** 电机FOC控制各阶段处理过程数据********************************************/

typedef struct T_MotorProcessData

{

    uint16_t unId;                        /*电机ID */

    MotorStatorState stStatorState;       /* 定子状态 */

    MotorRotorState stRotorState;         /* 转子状态 */

    FocExecutionTarget stExecutionTarget; /* 执行状态 */

    FocSvpwmDuty stDuty;                  /* SVPWM占空比 */

    FocError enControllerState;           /* 最新状态 */

} MotorProcessData;

 

/********************************************通用处理流程********************************************/

typedef struct T_MotorFocController

{

    uint16_t unId;                                            /*电机ID */

    Foc_CurrentVoltageSamplingHandler fHandlerCurrentVoltage; /* 电流电压采样函数 */

    Foc_RotorAngleEstimateHandler fHandlerAngleEsimate;       /* 角度估计函数指针 */

    Foc_RotorSpeedEstimateHandler fHandlerSpeedEsimate;       /* 速度估计函数指针 */

    Foc_ClarkeHandler fHandlerClarke;                         /* Clarke变换函数指针 */

    Foc_CosSinHandler fHandlerCosSin;                         /* 三角函数计算函数指针 */

    Foc_ParkHandler fHandlerPark;                             /* Park变换函数指针 */

    Foc_CurrentExecutionHandler fHandlerCurrentExecution;     /* Id, Iq执行值计算函数指针 */

    Foc_VoltageExecutionHandler fHandlerVoltageExecution;     /* Vd, Vq执行值计算函数指针 */

    Foc_InvParkHandler fHandlerInvPark;                       /* 反Park变换函数指针 */

    Foc_SvpwmHandler fHandlerSvpwm;                           /* SVPWM计算函数指针 */

} MotorFocController;

 

typedef struct

{

    /* 输出 */

    float fSmcOut;   /* 当前时刻的SMC控制量 Iq_ref */

    float fSum;      /* 上一时刻的积分项 */

    float fError;    /* 当前误差项 (rad/s) */

    float OutputLim; /*输出限幅*/

    /* 电机参数 */

    float fJ;      /* 转动惯量 */

    float fB;      /* 摩擦系数 */

    float fLambda; /* 滑模面斜率 */

    float fK;      /* 非线性增益 */

    float fPhi;    /* 饱和函数边界层 */

    float fQ;      /* 指数趋近律 */

    float f1_D;    /* 根据电机参数计算1/D */

    /* 状态变量 */

    int32_t nLoopCounts; /* 循环递减轮数 */

    float fSlide;        /* 滑模面 */

    float fErrorLast;    /* 上一时刻误差 (rad/s) */

} SmcCtl;

 

/* 记录APP下发命令 */

typedef struct

{

  uint8_t unType;

  uint64_t unTime;

  uint64_t unCmdInterval;

} CmdRecord;

 

 

 

#endif

 

 


