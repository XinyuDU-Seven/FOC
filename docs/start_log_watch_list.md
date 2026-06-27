# 起步慢问题 Watch 数据清单

本文档用于排查座椅电机起步慢、拉锯、一点点挪动后才突然上速的问题。
当前 start log 只保留关键字段，减少 LiveWatch 需要导出的数组数量。

## 运行前设置

只抓 1 号电机：

```text
g_foc_start_log_filter_motor_id = 1
g_foc_start_log_reset = 1
```

只抓 0 号电机：

```text
g_foc_start_log_filter_motor_id = 0
g_foc_start_log_reset = 1
```

允许任意电机触发：

```text
g_foc_start_log_filter_motor_id = 255
g_foc_start_log_reset = 1
```

默认触发条件：

```text
abs(speed_ref) >= g_foc_start_log_trigger_ref_rpm       // 默认 300 rpm
abs(speed_fdb) <= g_foc_start_log_trigger_fdb_max_rpm   // 默认 300 rpm
```

## 需要一起发的标量

```text
g_foc_start_log_enable
g_foc_start_log_armed
g_foc_start_log_active
g_foc_start_log_stop
g_foc_start_log_filter_motor_id
g_foc_start_log_trigger_motor_id
g_foc_start_log_idx
g_foc_start_log_decim_ms
g_foc_start_log_trigger_ref_rpm
g_foc_start_log_trigger_fdb_max_rpm
g_foc_start_log_trigger_count
```

判断是否抓到数据：

```text
g_foc_start_log_idx > 0
g_foc_start_log_trigger_count > 0
```

## 必须导出的数组

按 `idx = 0 .. g_foc_start_log_idx - 1` 导出。
只分析 `g_foc_start_log_valid[idx] == 1` 的行。

```text
g_foc_start_log_valid
g_foc_start_log_motor_id
g_foc_start_log_t_ms
g_foc_start_log_ref_rpm
g_foc_start_log_ctrl_ref_rpm
g_foc_start_log_fdb_rpm
g_foc_start_log_ctrl_fdb_rpm
g_foc_start_log_err_rpm
g_foc_start_log_iq_ref_mA
g_foc_start_log_iq_mA
g_foc_start_log_current_peak_mA
g_foc_start_log_vq_mV
g_foc_start_log_vbus_mV
g_foc_start_log_duty_max
g_foc_start_log_hall_sector
g_foc_start_log_sector_no_change_count
g_foc_start_log_edge_elapsed_us
g_foc_start_log_fault
g_foc_start_log_flags
g_foc_start_log_lift_extra_mA
g_foc_start_log_iq_slew_limited_mA
```

## 推荐 CSV 表头

```text
idx,valid,motor_id,t_ms,ref_rpm,ctrl_ref_rpm,fdb_rpm,ctrl_fdb_rpm,err_rpm,iq_ref_mA,iq_mA,current_peak_mA,vq_mV,vbus_mV,duty_max,hall_sector,sector_no_change_count,edge_elapsed_us,fault,flags,lift_extra_mA,iq_slew_limited_mA
```

## flags 位定义

```text
bit0  0x01  direction_ccw
bit1  0x02  speed_ref_ramp_active
bit2  0x04  lift_current_limit_active
bit3  0x08  low_speed_torque_active
bit4  0x10  low_speed_iq_slew_active
bit5  0x20  not_speed_control_source
bit6  0x40  fault_present
```

## 字段用途

速度跟随：

```text
ref_rpm
ctrl_ref_rpm
fdb_rpm
ctrl_fdb_rpm
err_rpm
flags bit1
```

用于判断参考速度是否已经给到、速度斜坡是否太慢、反馈速度是否卡在低速。

扭矩和限流：

```text
iq_ref_mA
iq_mA
current_peak_mA
flags bit2
flags bit3
flags bit4
lift_extra_mA
iq_slew_limited_mA
```

用于判断速度环是否给足 q 轴电流、抬升限流是否生效、q 轴电流斜率是否限制了起步。

电压和占空比：

```text
vq_mV
vbus_mV
duty_max
```

用于判断是否接近电压或占空比能力上限。

Hall 和低速边沿：

```text
hall_sector
sector_no_change_count
edge_elapsed_us
```

用于判断是否长时间没有 Hall 边沿，或者负载下出现卡滞趋势。

## 发送数据建议

优先发送完整 `0 .. g_foc_start_log_idx - 1` 数据。
如果导出太麻烦，先发送前 120 行和最后 120 行。

同时说明测试条件：

```text
motor_id:
方向: 抬升/下降
目标速度:
是否带载:
按键持续时间:
主观现象: 拉锯/慢起/机械声/突然上速
```
