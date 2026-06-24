
/**

 * @file FOC_ExternalInterface.h

 * @author FocTeam (FocTeam@byd.com)

 * @brief 所有对外部（FocDriver以外）提供的接口的统一声明定义

 * @version 1.0.0

 * @date 2025-07-24

 *

 * @copyright Copyright (c) 2025

 *

 */

#ifndef _FOC_EXTERNAL_INTERFACE_H_

#define _FOC_EXTERNAL_INTERFACE_H_

 

#include "FOC_DataType.h"

 

/*******************************************************************************************

  函数名称:  Foc_AlgorithmControlCallback

  函数功能:  电机的Foc中断函数，基于电流环的控制目标完成三相电机占空比输出

  输入参数：  无

  输出参数： 无

  返回值:    无

 *******************************************************************************************/

void Foc_AlgorithmControlCallback(void);

 

/*******************************************************************************************

  函数名称:  Foc_Init

  函数功能:   FOC电流环控制模块初始化，主要是配置霍尔中断回调函数，使用FOC模块提供的一切API接口前必须先被执行

  输入参数： 无

  输出参数： 无

  返回值:    无

 *******************************************************************************************/

void Foc_Init(void);

 

/*******************************************************************************************

  函数名称:  Foc_EnableFocControl

  函数功能:  激活电机的FOC电流环控制输出

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_EnableFocControl(uint8_t unId);

 

/*******************************************************************************************

  函数名称:  Foc_DisableFocControl

  函数功能:  停止电机的FOC电流环控制输出

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_DisableFocControl(uint8_t unId);

 

/*******************************************************************************************

  函数名称:  Foc_SetCurrentReference

  函数功能:  设置电机以电流控制闭环控制运行，d，q轴目标参考电流

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

            fId：d轴目标参考电流，单位A

            fIq：q轴目标参考电流，单位A

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_SetCurrentReference(uint8_t unId, float fId, float fIq);

FocError Foc_SetVoltageReference(uint8_t unId, float fVd, float fVq);

 

/*******************************************************************************************

  函数名称:  Foc_SetHybridControlReference

  函数功能:  无刷电机混合控制接口

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

            unMode: 控制模式，0: Vq比例+速度目标控制， 1: 速度闭环控制，2: Vq比例+电流目标控制， 3:

电流闭环控制，4：Vq目标控制 1) unMode == 0，Vq比例+速度目标控制： unParam1： 方向，0：无方向，1: 正方向，2:反方向

                unParam2： 输入电压Vq的比例，0~10000表示0%~100%

                unParam3： 输入电压Vq，mV

                unParam4： 输入参考速度Speed, RPM

                unParam5： 无效参数

            2) unMode == 1，速度闭环控制：

                unParam1： 方向，0：无方向，1: 正方向，2:反方向

                unParam2： 无效参数

                unParam3： 无效参数

                unParam4： 输入参考速度Speed, RPM

                unParam5： 无效参数

            3) unMode == 2，Vq比例+电流目标控制：

                unParam1： 方向，0：无方向，1: 正方向，2:反方向

                unParam2： 输入电压Vq的比例，0~10000表示0%~100%

                unParam3： 输入电压Vq，mV

                unParam4： 无效参数

                unParam5： 输入参考电流Iq, mA

            4) unMode == 3，电流闭环控制：

                unParam1： 方向，0：无方向，1: 正方向，2:反方向

                unParam2： 无效参数

                unParam3： 无效参数

                unParam4： 无效参数

                unParam5： 输入参考电流Iq, mA

            5) unMode == 4，Vq目标控制：

                unParam1： 方向，0：无方向，1: 正方向，2:反方向

                unParam2： 无效参数

                unParam3： 输入电压Vq，mV

                unParam4： 无效参数

                unParam5： 无效参数

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

*******************************************************************************************/

FocError Foc_SetHybridControlReference(uint8_t unId, uint8_t unMode, uint16_t unParam1, uint16_t unParam2,

                                       uint16_t unParam3, uint16_t unParam4, uint16_t unParam5);

 

/*******************************************************************************************

  函数名称:  Foc_SetSpeedReference

  函数功能:  设置速度闭环运行电机

  输入参数： unId：电机编号，0：水平无刷电机  1：坐盆无刷电机

            fSpeed：电机目标转速，RPM单位

  返回值:    FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_SetSpeedReference(uint8_t unId, float fSpeed);

FocError Foc_SetTorqueReference(uint8_t unId, float fTorque);

FocError Foc_SetIFReference(uint8_t unId, float fIq, float fSpeed);

FocError Foc_SetVFReference(uint8_t unId, float fVq, float fSpeed);

 

/*******************************************************************************************

  函数名称:  Foc_GetMotorFullParameters

  函数功能:  获取电机当前状态

  输入参数: unId：设备号，0：水平无刷电机  1：坐盆无刷电机

  输出参数: pstMotorFullStates：当前电机状态的所有参数的结构体指针

  返回值:  FOC_SUCCESS--返回成功

             其他--错误码参看FOC_DataType.h

 *******************************************************************************************/

FocError Foc_GetAngleAndSpeed(uint8_t unId, float *pfThetaElec, float *pfSpeed);

FocError Foc_GetMotorFullParameters(uint8_t unId, MotorFullStates *pstMotorFullStates);

 

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

FocError Foc_GetMotorNum(uint8_t unCarConfigID, uint8_t unSeatID, uint8_t unMotorID, uint8_t *punMotorNum);

FocError Foc_ReadMotorHallStates(uint8_t unId, int16_t *pstHallStatesOffset);

FocError Foc_WriteMotorHallStates(uint8_t unId, int16_t *pstHallStatesOffset, int16_t nHallDistanceOffset);

 

 

void Foc_TestCase(void);

#endif

 


