
#include "FOC_ExternalInterface.h"

#include "foc_api.h"

#ifdef __ICCARM__
#define FOC_AI_DEBUG_ROOT __root
#else
#define FOC_AI_DEBUG_ROOT
#endif

FOC_AI_DEBUG_ROOT volatile uint32_t g_foc_ai_callback_count = 0U;

/*******************************************************************************************

  函数名称:  Foc_AlgorithmControlCallback

  函数功能:  电机的Foc中断函数，基于电流环的控制目标完成三相电机占空比输出

  输入参数：  无

  输出参数： 无

  返回值:    无

 *******************************************************************************************/

void Foc_AlgorithmControlCallback_AI(void){

  g_foc_ai_callback_count++;

  FOC_MainLoop();

}

 

/*******************************************************************************************

  函数名称:  Foc_Init

  函数功能:   FOC电流环控制模块初始化，主要是配置霍尔中断回调函数，使用FOC模块提供的一切API接口前必须先被执行

  输入参数： 无

  输出参数： 无

  返回值:    无

 *******************************************************************************************/

void Foc_Init_AI(void){

  FOC_Config_t config;

  memset(&config,0,sizeof(FOC_Config_t));

  config.motor.pole_pairs = 4;

  config.motor.rs = 0.65;

  config.motor.ls_d = 0.00007;

  config.motor.ls_q = 0.00007;

  config.motor.v_bus = 12;

  config.motor.max_speed_rpm = 4000;

  config.motor.max_current_a = 5;

  config.current_d_pid.kp = 0.1;

  config.current_d_pid.ki = 10;

  config.current_d_pid.kd = 0;

  config.current_d_pid.out_max = 10;

  config.current_d_pid.out_min = -10;

  config.current_q_pid.kp = 0.1;

  config.current_q_pid.ki = 10;

  config.current_q_pid.kd = 0;

  config.current_q_pid.out_max = 10;

  config.current_q_pid.out_min = -10;

  config.speed_pid.kp = 0.005f;

  config.speed_pid.ki = 0.0005f;

  config.speed_pid.kd = 0.0f;

  config.speed_pid.out_max = config.motor.max_current_a;

  config.speed_pid.out_min = -config.motor.max_current_a;

  FOC_Init(&config);

  FOC_Start();

}

 

/*******************************************************************************************

  函数名称:  Foc_EnableFocControl

  函数功能:  激活电机的FOC电流环控制输出

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_EnableFocControl_AI(uint8_t unId){

    (void)unId;

    FOC_Start();

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_DisableFocControl

  函数功能:  停止电机的FOC电流环控制输出

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_DisableFocControl_AI(uint8_t unId){

    (void)unId;

    FOC_Stop();

    return 0;

}

/*******************************************************************************************

  函数名称:  Foc_SetCurrentReference

  函数功能:  设置电机以电流控制闭环控制运行，d，q轴目标参考电流

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

            fId：d轴目标参考电流，单位A

            fIq：q轴目标参考电流，单位A

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_SetCurrentReference_AI(uint8_t unId, float fId, float fIq){

    (void)unId;

    (void)fId;

    (void)fIq;

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_SetHybridControlReference

  函数功能:  无刷电机混合控制接口

   *******************************************************************************************/

FocError Foc_SetHybridControlReference_AI(uint8_t unId, uint8_t unMode, uint16_t unParam1, uint16_t unParam2,

                                       uint16_t unParam3, uint16_t unParam4, uint16_t unParam5)

{

    (void)unId;

    (void)unMode;

    (void)unParam1;

    (void)unParam2;

    (void)unParam3;

    (void)unParam4;

    (void)unParam5;

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_SetSpeedReference

  函数功能:  设置速度闭环运行电机

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

            fSpeed：电机目标转速，RPM单位

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_SetSpeedReference_AI(uint8_t unId, float fSpeed){

    (void)unId;

    FOC_SetSpeedRef(fSpeed);

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_GetMotorFullParameters

  函数功能:  获取电机当前状态

  输入参数: unId：设备号，0：水平无刷电机  1：坐盆无刷电机

  输出参数: pstMotorFullStates：当前电机状态的所有参数的结构体指针

  返回值:  FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_GetMotorFullParameters_AI(uint8_t unId, MotorFullStates *pstMotorFullStates){

    (void)unId;

    (void)pstMotorFullStates;

    return 0;

}

 

/*******************************************************************************************

  函数名称:  Foc_GetMotorNum

  函数功能:  FOC电机号接口函数

  输入参数: unCarConfigID：车型ID, SNHB暂定为1, 其余未定义

           unSeatID：座椅ID，副驾座椅CPLTSEAT

           unMotorID：电机ID，水平电机：LEVELMOTOR，坐盆电机：BIDETMOTOR

  输出参数: punMotorNum: 电机号指针

  返回值:  FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_GetMotorNum_AI(uint8_t unCarConfigID, uint8_t unSeatID, uint8_t unMotorID, uint8_t *punMotorNum){

    (void)unCarConfigID;

    (void)unSeatID;

    return 0;

}

 

float gfSpeedTarget = 0;

uint8_t gunCtrl = 0;

void Foc_TestCase(void){

  uint8_t unId = 0;

  if(gunCtrl == 1){

    Foc_EnableFocControl(unId);

    Foc_SetSpeedReference(unId, gfSpeedTarget);

  }else{

    Foc_DisableFocControl(unId);

  }

}

 

 

 


