
/**

 * @file foc_api.c

 * @brief FOC 对外 API 实现 — 薄封装层，委托给 foc_core

 */

 

#include <stddef.h>

#include "foc_api.h"

#include "foc_core.h"

 

int FOC_Init(const FOC_Config_t *config)

{

    return FOC_Core_Init(config);

}

 

int FOC_DeInit(void)

{

    return FOC_Core_DeInit();

}

 

int FOC_Start(void)

{

    return FOC_Core_Start();

}

 

int FOC_Stop(void)

{

    return FOC_Core_Stop();

}

 

int FOC_SetSpeedRef(float rpm)

{

    return FOC_Core_SetSpeedRef(rpm);

}

int FOC_SetCurrentRef(float id, float iq)

{

    return FOC_Core_SetCurrentRef(id, iq);

}

 

int FOC_SetDirection(FOC_Dir_e dir)

{

    return FOC_Core_SetDirection(dir);

}

 

int FOC_GetSpeed(float *rpm)

{

    if (rpm == NULL) return FOC_ERR;

    const FOC_Context_t *ctx = FOC_Core_GetContext();

    *rpm = ctx->speed_fdb;

    return FOC_OK;

}

 

int FOC_GetState(FOC_State_e *state)

{

    if (state == NULL) return FOC_ERR;

    const FOC_Context_t *ctx = FOC_Core_GetContext();

    *state = ctx->state;

    return FOC_OK;

}

 

int FOC_SetSpeedPID(const FOC_PID_Params_t *pid)

{

    if (pid == NULL) return FOC_ERR;

    return FOC_Core_SetSpeedPID(pid);

}

 

int FOC_SetCurrentPID(const FOC_PID_Params_t *pid)

{

    if (pid == NULL) return FOC_ERR;

    /* d轴和q轴使用相同参数 */

    return FOC_Core_SetCurrentPID(pid, pid);

}

 

int FOC_GetFaultCode(FOC_Fault_e *fault)

{

    if (fault == NULL) return FOC_ERR;

    const FOC_Context_t *ctx = FOC_Core_GetContext();

    *fault = ctx->fault;

    return FOC_OK;

}

 

int FOC_ClearFault(void)

{

    return FOC_Core_ClearFault();

}

 

void FOC_MainLoop(void)

{

    FOC_Core_MainLoop();

}

 

 


