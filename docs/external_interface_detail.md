# FOC External 接口详细说明

本文说明当前仓库中 `inc/FOC_ExternalInterface.h`、`inc/FOC_ExternalInterface_NoAI.h` 对应用层暴露的接口，以及 `src/FOC_ExternalInterface_AI.c` 中的实际处理逻辑。

## 1. 总体约定

### 1.1 接口文件

- 对外声明文件：`inc/FOC_ExternalInterface.h`
- 不带 AI 的对外声明文件：`inc/FOC_ExternalInterface_NoAI.h`
- 当前实际实现文件：`src/FOC_ExternalInterface_AI.c`
- FOC 核心运行文件：`src/foc_core.c`

当前普通接口 `Foc_xxx()` 都是对 `Foc_xxx_AI()` 的一层包装，例如 `Foc_SetHybridControlReference()` 直接调用 `Foc_SetHybridControlReference_AI()`。

### 1.2 电机号约定

当前 FOC 核心只支持两个物理电机：

| FOC 物理电机号 | 当前语义 |
| --- | --- |
| `0` | 水平电机 / Level motor |
| `1` | 坐盆电机 / Bidet motor |

应用层的座椅子电机 ID 和 FOC 物理电机号不是同一个概念。当前 `Foc_GetMotorNum()` 做应用层 ID 到 FOC 物理 ID 的映射：

| 应用层子电机 ID | 默认映射到 FOC 物理电机号 | Watch 可调变量 |
| --- | --- | --- |
| `g_foc_app_level_motor_id = 1` | `g_foc_app_level_foc_motor_id = 0` | 水平电机 |
| `g_foc_app_bidet_motor_id = 5` | `g_foc_app_bidet_foc_motor_id = 1` | 坐盆电机 |

`Foc_GetMotorNum()` 还允许应用层直接传 `0` 或 `1`，会原样返回 `0` 或 `1`。除此之外的应用层 ID 会返回 `FOC_MOTOR_ID_INVALID`。

重要：除 `Foc_GetMotorNum()` 外，控制、状态、Hall 相关接口都应该传入 FOC 物理电机号 `0/1`，也就是 `Foc_GetMotorNum()` 的输出值。当前 `Foc_SetSpeedReference()`、`Foc_SetHybridControlReference()` 等接口的入口只检查 `unId < 200`，但底层 `FOC_Core_SelectMotor()` 遇到大于等于 `2` 的物理电机号会回退到 `0`。所以如果应用层误把子电机 ID `5` 直接传给控制接口，实际可能会控制到 FOC 0 号电机。

建议应用层统一流程：

```c
uint8_t foc_motor_id;
FocError err;

err = Foc_GetMotorNum(car_id, seat_id, app_sub_motor_id, &foc_motor_id);
if (err == FOC_SUCCESS) {
    (void)Foc_EnableFocControl(foc_motor_id);
    (void)Foc_SetHybridControlReference(foc_motor_id, 1U, 1U, 0U, 0U, 2000U, 0U);
}
```

### 1.3 方向约定

`Foc_SetHybridControlReference()` 中 `unParam1` 表示应用层方向：

| `unParam1` | 含义 | 内部处理 |
| --- | --- | --- |
| `0` | 无方向 | 只有目标幅值为 `0` 时合法，输出目标为 `0` |
| `1` | 正方向 | 目标值取正 |
| `2` | 反方向 | 目标值取负 |

如果目标幅值不为 `0`，但方向不是 `1` 或 `2`，返回 `FOC_INVALID_DIRECITON`。

### 1.4 常见返回值

| 返回值 | 含义 | 当前常见触发条件 |
| --- | --- | --- |
| `FOC_SUCCESS` | 成功 | 参数合法且底层处理成功 |
| `FOC_POINTER_NULL` | 空指针 | 输出指针为 `NULL` |
| `FOC_MOTOR_ID_INVALID` | 电机号非法 | `Foc_GetMotorNum()` 映射失败，或 Hall 接口传入非物理号 |
| `FOC_INVALID_DIRECITON` | 方向非法 | 非零目标下方向不是 `1/2` |
| `FOC_INPUT_PARAMETER_INVALID` | 输入参数非法 / 当前未实现 | 非法模式，或未实现接口被调用 |
| `FOC_VOLTAGE_SETTING_RANGE_INVALID` | 电压超范围 | `Vd/Vq` 绝对值大于 `20V` |
| `FOC_MOTOR_DISABLED` | 底层忙或故障导致不可启动 | `FOC_Start()` / `FOC_Stop()` 返回 `FOC_BUSY` 或 `FOC_FAULT` |
| `FOC_MOTOR_STUCKED` | 堵转或速度掉落类故障 | 内部 fault 映射 |
| `FOC_HALL_STATES_INVALID` | Hall 状态异常 | 内部 Hall fault 映射 |
| `FOC_PHASE_ABC_SQRT_CURRENT_EXCEED` | 电流超限 | 内部过流 fault 映射 |
| `FOC_PHASE_VOLTAGE_HIGH_EXCEED` | 过压 | 内部过压 fault 映射 |
| `FOC_PHASE_VOLTAGE_LOW_EXCEED` | 欠压 | 内部欠压 fault 映射 |
| `FOC_ABNORMAL_INTERVAL_TIME` | 速度反馈掉落类异常 | 内部 speed feedback drop fault 映射 |

## 2. 初始化和主循环接口

### 2.1 `void Foc_Init(void)`

应用层输入：无。

内部处理：

- 构造 `FOC_Config_t config`。
- 设置电机极对数 `4`、母线电压 `12V`、最大速度 `4000rpm`、最大电流 `5A`。
- 设置 d/q 轴电流环 PID 和速度环 PID。
- 调用 `FOC_Init(&config)` 初始化 FOC 核心。

返回值：无。

使用要求：应用层在调用其他 FOC API 前应先调用一次。

### 2.2 `void Foc_AlgorithmControlCallback(void)`

应用层输入：无。

内部处理：

- 调用 `Foc_AlgorithmControlCallback_AI()`。
- `g_foc_ai_callback_count++`，可在 Watch 中观察主循环是否持续运行。
- 调用 `FOC_MainLoop()`，最终会轮询执行当前两个 FOC 物理电机的控制循环。

返回值：无。

使用要求：该函数应放在 FOC 周期回调或定时中断主循环中。如果这个回调不运行，已经设置的速度、电流目标不会被实际执行。

## 3. 电机号映射接口

### 3.1 `FocError Foc_GetMotorNum(uint8_t unCarConfigID, uint8_t unSeatID, uint8_t unMotorID, uint8_t *punMotorNum)`

应用层输入：

- `unCarConfigID`：车型配置 ID。当前实现未使用。
- `unSeatID`：座椅 ID。当前实现未使用。
- `unMotorID`：应用层子电机 ID，例如 Level 是 `1`，Bidet 是 `5`。
- `punMotorNum`：输出指针，用于返回 FOC 物理电机号。

内部处理：

- 记录调试变量：
  - `g_foc_last_get_motor_num_app_id = unMotorID`
  - `g_foc_last_get_motor_num_foc_id = 0xFF`
  - `g_foc_last_get_motor_num_err = 返回错误码`
- 如果 `punMotorNum == NULL`，返回 `FOC_POINTER_NULL`。
- 调用内部映射：
  - `unMotorID == g_foc_app_level_motor_id` 时，输出 `g_foc_app_level_foc_motor_id`。
  - `unMotorID == g_foc_app_bidet_motor_id` 时，输出 `g_foc_app_bidet_foc_motor_id`。
  - `unMotorID < 2` 时，认为应用层已经传了 FOC 物理号，原样输出。
  - 其他情况返回 `FOC_MOTOR_ID_INVALID`。
- 输出值必须小于 `2`，否则返回 `FOC_MOTOR_ID_INVALID`。

返回值：

- 成功：`FOC_SUCCESS`，`*punMotorNum` 为 `0` 或 `1`。
- 失败：`FOC_POINTER_NULL` 或 `FOC_MOTOR_ID_INVALID`。

Watch 建议：

- `g_foc_last_get_motor_num_app_id`
- `g_foc_last_get_motor_num_foc_id`
- `g_foc_last_get_motor_num_err`

## 4. 使能和停止接口

### 4.1 `FocError Foc_EnableFocControl(uint8_t unId)`

应用层输入：

- `unId`：FOC 物理电机号，必须使用 `0/1`，建议来自 `Foc_GetMotorNum()` 的输出。

内部处理：

- 选择目标电机：`FOC_Core_SelectMotor(unId)`。
- 更新 `g_foc_selected_motor_id`。
- 如果当前电机已经是 `FOC_STATE_RUNNING`，直接返回成功。
- 如果当前电机是 `FOC_STATE_FAULT`，按 fault 位映射成对外错误码返回。
- 其他状态下调用 `FOC_Start()` 启动该电机的 FOC 输出。

返回值：

- `FOC_SUCCESS`：已经运行或启动成功。
- `FOC_MOTOR_ID_INVALID`：入口 ID 不合法。
- fault 映射值：当前电机在 fault 状态。
- `FOC_MOTOR_DISABLED`：底层启动返回 busy/fault。
- `FOC_INPUT_PARAMETER_INVALID`：底层返回其他异常。

注意：`SetSpeedReference` 或 `SetHybridControlReference` 只写目标，不等价于启动 PWM。应用层通常应在按键动作开始时先调用 `Foc_EnableFocControl()`。

### 4.2 `FocError Foc_DisableFocControl(uint8_t unId)`

应用层输入：

- `unId`：FOC 物理电机号。

内部处理：

- 选择目标电机。
- 清除内部测试/自动模式标志。
- 调用 `FOC_Stop()` 停止 FOC 输出。

返回值：

- `FOC_SUCCESS`：停止成功。
- `FOC_MOTOR_ID_INVALID`：入口 ID 不合法。
- `FOC_MOTOR_DISABLED`：底层 stop 返回 busy/fault。
- `FOC_INPUT_PARAMETER_INVALID`：底层返回其他异常。

注意：这是关闭 FOC 输出，不是座椅舒适性意义上的软减速。若应用层需要“松手软停”，应先通过速度目标平滑降到接近 0，再按策略决定是否 disable。

## 5. 控制目标设置接口

### 5.1 `FocError Foc_SetHybridControlReference(uint8_t unId, uint8_t unMode, uint16_t unParam1, uint16_t unParam2, uint16_t unParam3, uint16_t unParam4, uint16_t unParam5)`

这是当前推荐给应用层使用的混合控制入口。它是本接口层里允许调用其他 set reference 接口的组合接口。

应用层输入：

- `unId`：FOC 物理电机号。
- `unMode`：控制模式。
- `unParam1`：方向，`0` 无方向，`1` 正方向，`2` 反方向。
- `unParam2`：当前实现中未使用。
- `unParam3`：Vq 目标，单位 mV，仅 mode 4 会读取。
- `unParam4`：速度目标，单位 rpm，mode 0/1 会读取。
- `unParam5`：Iq 目标，单位 mA，mode 2/3 会读取。

当前模式处理：

| `unMode` | 说明 | 当前实际处理 |
| --- | --- | --- |
| `0` | Vq 比例 + 速度目标控制 | 当前等同速度模式，忽略 `unParam2/unParam3`，用 `unParam4` 设置速度 |
| `1` | 速度闭环控制 | 用 `unParam1` 和 `unParam4` 生成带符号 rpm，然后调用 `Foc_SetSpeedReference()` |
| `2` | Vq 比例 + 电流目标控制 | 当前等同电流模式，忽略 `unParam2/unParam3/unParam4`，用 `unParam5` 设置 Iq |
| `3` | 电流闭环控制 | 用 `unParam1` 和 `unParam5` 生成带符号 Iq，单位从 mA 转 A，然后调用 `Foc_SetCurrentReference()` |
| `4` | Vq 目标控制 | 用 `unParam1` 和 `unParam3` 生成带符号 Vq，单位从 mV 转 V，然后调用 `Foc_SetVoltageReference()` |

内部处理：

- 先选择电机。
- 根据 mode 生成带符号目标：
  - 目标幅值为 `0` 时，方向可以为 `0/1/2`，最终目标为 `0`。
  - 目标幅值非 `0` 时，方向 `1` 输出正目标，方向 `2` 输出负目标。
  - 目标幅值非 `0` 且方向不是 `1/2`，返回 `FOC_INVALID_DIRECITON`。
- 根据 mode 分发到速度、电流或电压接口。

返回值：

- 成功：`FOC_SUCCESS`。
- 非法 mode：`FOC_INPUT_PARAMETER_INVALID`。
- 非法方向：`FOC_INVALID_DIRECITON`。
- 下游接口返回的错误码。

常用调用：

```c
/* 正向 2000 rpm */
Foc_SetHybridControlReference(foc_motor_id, 1U, 1U, 0U, 0U, 2000U, 0U);

/* 反向 2000 rpm */
Foc_SetHybridControlReference(foc_motor_id, 1U, 2U, 0U, 0U, 2000U, 0U);

/* 目标速度 0，全 0 调用当前会落入 mode 0，并设置速度目标 0 */
Foc_SetHybridControlReference(foc_motor_id, 0U, 0U, 0U, 0U, 0U, 0U);
```

注意：mode 4 目前会调用 `Foc_SetVoltageReference()`，但电压接口当前未实现实际控制，所以合法电压范围内会返回 `FOC_INPUT_PARAMETER_INVALID`。

### 5.2 `FocError Foc_SetSpeedReference(uint8_t unId, float fSpeed)`

应用层输入：

- `unId`：FOC 物理电机号。
- `fSpeed`：目标转速，单位 rpm，可为正、负或 0。

内部处理：

- 选择电机。
- 如果当前电机处于 fault 状态，返回 fault 映射错误码。
- 清除内部测试/自动模式。
- 调用 `FOC_SetSpeedRef(fSpeed)` 写入速度目标。

返回值：

- `FOC_SUCCESS`：速度目标写入成功。
- `FOC_MOTOR_ID_INVALID`：电机号非法。
- fault 映射错误码：当前电机处于 fault。
- `FOC_MOTOR_DISABLED` 或 `FOC_INPUT_PARAMETER_INVALID`：底层设置失败。

注意：该接口本身不调用 `FOC_Start()`。如果电机处于 idle，速度目标可以写入上下文，但不会自动产生 PWM 输出。

### 5.3 `FocError Foc_SetCurrentReference(uint8_t unId, float fId, float fIq)`

应用层输入：

- `unId`：FOC 物理电机号。
- `fId`：d 轴目标电流，单位 A。
- `fIq`：q 轴目标电流，单位 A。q 轴电流通常对应电磁转矩方向和大小。

内部处理：

- 选择电机。
- 如果当前电机处于 fault 状态，返回 fault 映射错误码。
- 清除内部测试/自动模式。
- 调用 `FOC_SetCurrentRef(fId, fIq)`。

返回值：

- `FOC_SUCCESS`：电流目标写入成功。
- `FOC_MOTOR_ID_INVALID`：电机号非法。
- fault 映射错误码：当前电机处于 fault。
- `FOC_MOTOR_DISABLED` 或 `FOC_INPUT_PARAMETER_INVALID`：底层设置失败。

### 5.4 `FocError Foc_SetVoltageReference(uint8_t unId, float fVd, float fVq)`

应用层输入：

- `unId`：FOC 物理电机号。
- `fVd`：d 轴电压目标，单位 V。
- `fVq`：q 轴电压目标，单位 V。

内部处理：

- 检查 `unId < 200`。
- 检查 `|fVd| <= 20V` 且 `|fVq| <= 20V`。
- 当前没有调用底层电压控制。

返回值：

- `FOC_MOTOR_ID_INVALID`：电机号非法。
- `FOC_VOLTAGE_SETTING_RANGE_INVALID`：电压超出 `[-20V, 20V]`。
- `FOC_INPUT_PARAMETER_INVALID`：参数范围合法，但该接口当前未实现实际控制。

### 5.5 `FocError Foc_SetTorqueReference(uint8_t unId, float fTorque)`

应用层输入：

- `unId`：FOC 物理电机号。
- `fTorque`：目标扭矩，单位 N·m。

内部处理：

- 仅检查 `unId < 200`。
- 当前未实现扭矩闭环控制。

返回值：

- `FOC_MOTOR_ID_INVALID`：电机号非法。
- `FOC_INPUT_PARAMETER_INVALID`：接口当前未实现。

### 5.6 `FocError Foc_SetIFReference(uint8_t unId, float fIq, float fSpeed)`

应用层输入：

- `unId`：FOC 物理电机号。
- `fIq`：q 轴电流目标，单位 A。
- `fSpeed`：目标速度，单位 rpm。

内部处理：

- 仅检查 `unId < 200`。
- 当前未实现 I/F 控制。

返回值：

- `FOC_MOTOR_ID_INVALID`：电机号非法。
- `FOC_INPUT_PARAMETER_INVALID`：接口当前未实现。

### 5.7 `FocError Foc_SetVFReference(uint8_t unId, float fVq, float fSpeed)`

应用层输入：

- `unId`：FOC 物理电机号。
- `fVq`：q 轴电压目标，单位 V。
- `fSpeed`：目标速度，单位 rpm。

内部处理：

- 检查 `unId < 200`。
- 检查 `|fVq| <= 20V`。
- 当前未实现 V/F 控制。

返回值：

- `FOC_MOTOR_ID_INVALID`：电机号非法。
- `FOC_VOLTAGE_SETTING_RANGE_INVALID`：电压超范围。
- `FOC_INPUT_PARAMETER_INVALID`：接口当前未实现。

## 6. 状态读取接口

### 6.1 `FocError Foc_GetAngleAndSpeed(uint8_t unId, float *pfThetaElec, float *pfSpeed)`

应用层输入：

- `unId`：FOC 物理电机号。
- `pfThetaElec`：输出指针，返回电角度。
- `pfSpeed`：输出指针，返回机械速度 rpm。

内部处理：

- 如果任一输出指针为空，返回 `FOC_POINTER_NULL`。
- 选择电机。
- 从当前电机 `FOC_Context_t` 中读取：
  - `ctx->theta_e` 写入 `*pfThetaElec`。
  - `ctx->speed_fdb` 经方向符号转换后写入 `*pfSpeed`。

返回值：

- `FOC_SUCCESS`：读取成功。
- `FOC_POINTER_NULL`：输出指针为空。
- `FOC_MOTOR_ID_INVALID`：电机号非法。

### 6.2 `FocError Foc_GetMotorFullParameters(uint8_t unId, MotorFullStates *pstMotorFullStates)`

应用层输入：

- `unId`：FOC 物理电机号。
- `pstMotorFullStates`：输出结构体指针。

内部处理：

- 如果结构体指针为空，返回 `FOC_POINTER_NULL`。
- 选择电机。
- `memset` 清零整个 `MotorFullStates`。
- 从当前 FOC context 填充关键字段。

当前填充字段：

| 字段 | 来源 / 含义 |
| --- | --- |
| `unId` | 输入的 `unId`，即 FOC 物理电机号 |
| `unHallState` | 当前 Hall raw 合成值，`h1 << 2 | h2 << 1 | h3` |
| `fSpeedMechEstimate` | 当前速度反馈，按内部方向转换正负号 |
| `fSpeedMechEstimateFiltered` | 当前滤波速度，按内部方向转换正负号 |
| `fThetaElec` | 当前电角度 `ctx->theta_e` |
| `fId` | d 轴实际电流 |
| `fIq` | q 轴实际电流 |
| `fIqRef` | q 轴目标电流，按内部方向转换正负号 |
| `fIa/fIb/fIc` | 三相电流 |
| `fUd/fUq` | d/q 轴电压 |
| `unNumberPoles` | 当前固定为 `4` 极对数 |
| `unUpdateTime` | Hall 更新时间；若没有 Hall 时间戳，则用当前时间 |
| `enFocState` | 当前 fault 映射后的 FOC 状态 |
| `unHeadIndexHall` | 当前置为 `-1` |
| `unHeadIndexAppHall` | 当前置为 `-1` |
| `unDirection` | 内部方向转应用层方向，CCW 返回 `2`，其他返回 `1` |

返回值：

- `FOC_SUCCESS`：读取成功。
- `FOC_POINTER_NULL`：输出结构体为空。
- `FOC_MOTOR_ID_INVALID`：电机号非法。

注意：该接口当前没有把应用层子电机 ID 反向填回 `unId`，`unId` 返回的是调用时传入的 FOC 物理电机号。

## 7. Hall 行程接口

### 7.1 `FocError Foc_ReadMotorHallStates(uint8_t unId, int16_t *pstHallStatesOffset)`

应用层输入：

- `unId`：FOC 物理电机号，必须是 `0/1`。
- `pstHallStatesOffset`：输出指针。

内部处理：

- 如果输出指针为空，返回 `FOC_POINTER_NULL`。
- 调用 `FOC_Core_ReadHallTravel(unId, pstHallStatesOffset)`。
- FOC 核心在临界区内计算：

```c
travel = g_foc_hall_travel_count[unId] + g_foc_hall_travel_offset[unId];
*pstHallStatesOffset = FOC_ClampI32ToI16(travel);
```

返回值：

- `FOC_SUCCESS`：读取成功。
- `FOC_POINTER_NULL`：输出指针为空。
- `FOC_MOTOR_ID_INVALID`：`unId` 不是 `0/1`。

含义：

- `g_foc_hall_travel_count[unId]` 是 FOC 根据 Hall 扇区变化累计的行程计数。
- `g_foc_hall_travel_offset[unId]` 是应用层写入绝对位置后形成的偏移。
- 读出的值是“当前累计 Hall 行程 + 应用层位置偏移”的结果。

### 7.2 `FocError Foc_WriteMotorHallStates(uint8_t unId, int16_t *pstHallStatesOffset, int16_t nHallDistanceOffset)`

应用层输入：

- `unId`：FOC 物理电机号，必须是 `0/1`。
- `pstHallStatesOffset`：输出指针，函数会把写入后的当前位置回填到该指针。
- `nHallDistanceOffset`：当前应用层认为的绝对 Hall 位置。虽然参数名带 Offset，但当前实现把它当作绝对位置 `hall_position` 使用，不是增量。

内部处理：

- 如果输出指针为空，返回 `FOC_POINTER_NULL`。
- 调用 `FOC_Core_WriteHallTravelOffset(unId, pstHallStatesOffset, nHallDistanceOffset)`。
- FOC 核心在临界区内执行：

```c
g_foc_hall_travel_offset[unId] = hall_position - g_foc_hall_travel_count[unId];
*pstHallStatesOffset = hall_position;
```

返回值：

- `FOC_SUCCESS`：写入成功。
- `FOC_POINTER_NULL`：输出指针为空。
- `FOC_MOTOR_ID_INVALID`：`unId` 不是 `0/1`。

使用语义：

- 该接口适合在应用层初始化、归零、位置同步时调用。
- 应用层传入“当前绝对位置”，FOC 计算内部 offset，让后续 `ReadMotorHallStates()` 返回值能继续对齐应用层位置。
- 它不会强制修改底层 Hall 累计计数，只是修改 offset。

## 8. 推荐应用层调用策略

### 8.1 按键按下启动

1. 应用层根据座椅对象拿到 `m_eSeatSubMotorID`。
2. 调用 `Foc_GetMotorNum()` 转成 FOC 物理电机号。
3. 调用 `Foc_EnableFocControl(foc_motor_id)`。
4. 调用 `Foc_SetHybridControlReference(foc_motor_id, 1, direction, 0, 0, speed_rpm, 0)`。

### 8.2 按键保持

如果目标速度、方向没有变化，不必每个应用层周期重复调用 sethybrid。FOC 周期回调会持续执行上一次写入的目标。

如果应用层监测到方向或目标速度变化，再调用 `Foc_SetHybridControlReference()` 更新目标。

### 8.3 按键松开

当前全 0 调用：

```c
Foc_SetHybridControlReference(foc_motor_id, 0U, 0U, 0U, 0U, 0U, 0U);
```

会进入 mode 0，并设置速度目标为 0。它不是 `Disable`，也不是完全无控制滑行，而是给速度环一个 0 目标。若座椅舒适性要求“软减速到接近 0”，应用层更适合先按斜坡逐步降低 `unParam4`，最后再决定是否调用 `Foc_DisableFocControl()`。

### 8.4 位置同步

应用层如果有自己的 Hall 行程累计值，应在初始化或归零时调用：

```c
int16_t foc_pos;
Foc_WriteMotorHallStates(foc_motor_id, &foc_pos, app_absolute_pos);
```

运行中读取：

```c
int16_t foc_pos;
Foc_ReadMotorHallStates(foc_motor_id, &foc_pos);
```

注意：这两个 Hall 接口都要求传 FOC 物理电机号，不做应用层 ID 映射。

## 9. Watch 排查变量

### 9.1 电机号映射

- `g_foc_app_level_motor_id`
- `g_foc_app_bidet_motor_id`
- `g_foc_app_level_foc_motor_id`
- `g_foc_app_bidet_foc_motor_id`
- `g_foc_last_get_motor_num_app_id`
- `g_foc_last_get_motor_num_foc_id`
- `g_foc_last_get_motor_num_err`
- `g_foc_selected_motor_id`

### 9.2 控制状态

- `g_foc_ctx[0].state`
- `g_foc_ctx[1].state`
- `g_foc_ctx[0].speed_ref`
- `g_foc_ctx[1].speed_ref`
- `g_foc_ctx[0].speed_fdb`
- `g_foc_ctx[1].speed_fdb`
- `g_foc_ctx[0].iq_ref`
- `g_foc_ctx[1].iq_ref`
- `g_foc_ctx[0].i_dq.q`
- `g_foc_ctx[1].i_dq.q`
- `g_foc_ctx[0].fault`
- `g_foc_ctx[1].fault`

### 9.3 Hall 行程

- `g_foc_hall_travel_count[0]`
- `g_foc_hall_travel_count[1]`
- `g_foc_hall_travel_offset[0]`
- `g_foc_hall_travel_offset[1]`
- `g_foc_ctx[0].hall_sector`
- `g_foc_ctx[1].hall_sector`

## 10. 当前限制和注意事项

1. `Foc_SetVoltageReference()`、`Foc_SetTorqueReference()`、`Foc_SetIFReference()`、`Foc_SetVFReference()` 当前未实现实际控制，合法参数下也会返回 `FOC_INPUT_PARAMETER_INVALID`。
2. `Foc_SetHybridControlReference()` 的 mode 0 和 mode 2 当前没有实现 Vq 比例前馈，`unParam2` 当前完全未使用。
3. 控制接口应传 FOC 物理电机号 `0/1`。不要把应用层子电机 ID 直接传给控制接口，否则可能被底层回退到 0 号电机。
4. `Foc_SetSpeedReference()` 和 `Foc_SetCurrentReference()` 当前不会因为 idle 直接报错，但也不会自动启动 PWM。需要应用层调用 `Foc_EnableFocControl()`。
5. `Foc_DisableFocControl()` 是关闭 FOC 输出，不是舒适性软停。
6. `Foc_WriteMotorHallStates()` 的第三个参数当前表示应用层绝对 Hall 位置，不是增量 offset。
