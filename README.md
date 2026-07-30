# FOC

This repository contains a Hall-sensor-based FOC motor control module for an embedded IAR project.

当前仓库主要保存 FOC 算法源码和头文件，目录结构保持简单，便于直接加入 IAR 工程编译。

## Project Layout

```text
inc/    Public headers, data types, configuration, HAL interface
src/    FOC core, observer, transforms, PID, SVPWM, protection, port layer
```

Key files:

```text
inc/foc_config.h      Compile-time defaults and protection thresholds
inc/foc_types.h       Common FOC data structures
inc/foc_hal_if.h      Hardware abstraction interface required by FOC
inc/foc_api.h         Unified public API for application code
inc/foc_core.h        Main FOC control API

src/foc_core.c        Main state machine and control loop
src/foc_observer.c    Hall decode, angle prediction, speed estimate, ADC parsing
src/foc_transform.c   Clarke / Park / inverse Park transforms
src/foc_svpwm.c       Seven-segment SVPWM duty generation
src/foc_pid.c         PID controller
src/foc_protection.c  Overcurrent / voltage / Hall / stall protection
src/foc_port.c        Hardware adaptation layer
```

## Control Flow

`FOC_Core_MainLoop()` is intended to run periodically at `FOC_CONTROL_FREQ_HZ`.

The current loop flow is:

```text
Read Hall / phase current / bus voltage raw data
Decode Hall sector and electrical angle
Predict electrical angle
Estimate speed
Convert ADC raw values to physical current / voltage
Clarke transform
Park transform
Speed loop -> iq reference
Current loop -> vd / vq
Inverse Park transform
SVPWM
PWM duty output
Protection check
Debug logging
```

The default control frequency is:

```c
#define FOC_CONTROL_FREQ_HZ 10000U
```

## Hall Sensor Mapping

The measured clockwise Hall sequence is:

```text
100 -> 110 -> 010 -> 011 -> 001 -> 101
```

The current lookup maps this sequence to continuous sectors:

```text
2 -> 3 -> 4 -> 5 -> 6 -> 1
```

The Hall sector table keeps sector center angles internally, but the control synchronization angle uses a configurable offset:

```c
#define FOC_HALL_ANGLE_OFFSET_RAD (-(FOC_PI / 6.0f))
```

This default changes the Hall synchronization point from sector center to sector entry edge, which is important for reducing d-axis current spikes during Hall-sector transitions.

After changing Hall wiring, motor phase order, or rotation direction, re-check:

```text
hall_raw
hall_sector
theta_hall
theta_ctrl
id_mA
iq_mA
phase current peak
```

## Protection Defaults

Main default protection values are defined in `inc/foc_config.h`:

```c
#define FOC_OVERCURRENT_THRESHOLD_A  10.0f
#define FOC_OVERVOLTAGE_THRESHOLD_V  15.0f
#define FOC_UNDERVOLTAGE_THRESHOLD_V 9.0f
```

For bring-up, use conservative current and speed limits first. Do not jump directly to high speed until Hall angle, current sampling polarity, and phase order are verified.

## Debug Logging

The project includes flattened debug log arrays for IAR Watch / Live Watch export. These are easier to copy as text than nested structures.

For loop timing diagnosis, watch `g_foc_prof_*` variables to confirm whether the 100us scheduler is missing control periods.

Useful variables:

```text
g_foc_log_stop
g_log_idx
g_log_fault_idx
g_log_seq
g_log_t_us
g_log_fault
g_log_state
g_log_direction
g_log_hall_raw
g_log_hall_sector
g_log_theta_hall
g_log_theta_pred
g_log_theta_ctrl
g_log_speed_ref_rpm
g_log_speed_fdb_rpm
g_log_speed_ctrl_fdb_rpm
g_log_sector_no_change_count
g_log_stall_counter
g_log_ia_mA
g_log_ib_mA
g_log_ic_mA
g_log_id_mA
g_log_iq_mA
g_log_id_ref_mA
g_log_iq_ref_mA
g_log_speed_error_boost_mA
g_log_current_peak_mA
g_log_vd_mV
g_log_vq_mV
g_log_vbus_mV
g_log_duty_a
g_log_duty_b
g_log_duty_c
```

When sending logs for analysis, do not export the whole buffer unless needed. Usually this window is enough:

```text
fault_idx - 24  ...  fault_idx + 3
```

Successful `Foc_SetHybridControlReference()` speed-mode calls also enable a
2500-point speed trace. It records one point after every 10 FOC callbacks.
The three signed rpm arrays are:

```text
g_foc_sethybrid_speed_log_target_rpm
g_foc_sethybrid_speed_log_actual_rpm
g_foc_sethybrid_speed_log_filtered_rpm
```

The valid range is `[0, g_foc_sethybrid_speed_log_idx)`. Samples are kept in
time order from oldest to newest. After 2500 points, each new sample discards
the oldest point and appends the latest point at index 2499. Set
`g_foc_sethybrid_speed_log_reset` to `1` to clear the arrays while logging
continues.

For testcase 3, the first 3s sine cycle can be exported from
`g_foc_test_case_first_cycle_log_*` after
`g_foc_test_case_first_cycle_log_done == 1`. The Hall diagnosis fields are:

```text
g_foc_test_case_first_cycle_log_hall_dt_us
g_foc_test_case_first_cycle_log_no_edge_elapsed_us
g_foc_test_case_first_cycle_log_hall_event_seq
g_foc_test_case_first_cycle_log_hall_debug
```

`hall_debug` bit layout:

```text
bits 0..2   raw Hall code: h1h2h3
bits 3..5   currently accepted Hall sector
bits 6..8   latest candidate Hall sector
bits 9..11  previous sector used by the apply filter
bit  12     latest candidate accepted
bits 13..15 reject reason: 0 none, 1 invalid, 2 non-adjacent, 3 min-time
```

Recommended testcase 3 row format:

```text
idx,t_ms,target_rpm,actual_rpm,filtered_rpm,iq_ref_mA,iq_mA,vbus_mV,no_edge_limit_rpm,sector_no_change,flags,hall_dt_us,no_edge_elapsed_us,hall_event_seq,hall_debug
```

Recommended compact row format:

```text
idx,seq,t_us,fault,state,direction,hall_raw,hall_sector,theta_hall,theta_ctrl,theta_pred,speed_ref,speed_fdb,speed_ctrl_fdb,sector_no_change,stall_counter,ia,ib,ic,id,iq,id_ref,iq_ref,speed_boost,ipeak,vd,vq,vbus,da,db,dc
```

Also include:

```text
target speed
actual symptom
direction
load condition
bus voltage
FOC_HALL_ANGLE_OFFSET_RAD
overcurrent threshold
```

## IAR Integration

Add the `inc/` directory to the compiler include path.

Add the required `src/*.c` files to the IAR project. The hardware-specific adaptation should be completed in `src/foc_port.c`, which implements the functions declared by `inc/foc_hal_if.h`.

The application should generally:

```c
FOC_Core_Init(&config);
FOC_Core_SetSpeedRef(target_rpm);
FOC_Core_Start();
```

Then call:

```c
FOC_Core_MainLoop();
```

from the control timer interrupt or an equivalent fixed-period control task.

## Git Workflow

After local changes:

```powershell
git status
git add .
git commit -m "Describe the change"
git push
```
