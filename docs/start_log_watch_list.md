# 起步慢问题 Watch 数据清单

本文档用于排查座椅电机起步慢、拉锯、一点点挪动后才突然上速的问题。
测试时只需要运行程序并按键，`g_foc_start_log_*` 会在低速起步阶段自动记录。

## 触发与控制变量

运行前建议先设置：

```text
g_foc_start_log_filter_motor_id = 1
g_foc_start_log_reset = 1
```

如果要抓 0 号电机，把 `g_foc_start_log_filter_motor_id` 设为 `0`。
如果要允许任意电机触发，把它设为 `255` / `0xFF`。

需要一起截图或导出的标量：

```text
g_foc_start_log_enable
g_foc_start_log_reset
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

重新抓一次：

```text
g_foc_start_log_reset = 1
```

## 必须导出的数组

按 `idx = 0 .. g_foc_start_log_idx - 1` 导出下列数组。
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
g_foc_start_log_id_mA
g_foc_start_log_iq_mA
g_foc_start_log_current_peak_mA
g_foc_start_log_vq_mV
g_foc_start_log_vbus_mV
g_foc_start_log_duty_a
g_foc_start_log_duty_b
g_foc_start_log_duty_c
g_foc_start_log_hall_raw
g_foc_start_log_hall_sector
g_foc_start_log_sector_no_change_count
g_foc_start_log_edge_elapsed_us
g_foc_start_log_theta_hall
g_foc_start_log_theta_pred
g_foc_start_log_theta_ctrl
g_foc_start_log_direction
g_foc_start_log_state
g_foc_start_log_fault
g_foc_start_log_ctrl_source
g_foc_start_log_ramp_active
g_foc_start_log_lift_active
g_foc_start_log_lift_extra_mA
g_foc_start_log_torque_active
g_foc_start_log_torque_mA
g_foc_start_log_iq_slew_active
g_foc_start_log_iq_slew_limited_mA
g_foc_start_log_q_ff_mV
g_foc_start_log_speed_boost_mA
```

## 推荐 CSV 表头

如果 Watch 工具能导出成表格，推荐按这个表头发回：

```text
idx,valid,motor_id,t_ms,ref_rpm,ctrl_ref_rpm,fdb_rpm,ctrl_fdb_rpm,err_rpm,iq_ref_mA,id_mA,iq_mA,current_peak_mA,vq_mV,vbus_mV,duty_a,duty_b,duty_c,hall_raw,hall_sector,sector_no_change_count,edge_elapsed_us,theta_hall,theta_pred,theta_ctrl,direction,state,fault,ctrl_source,ramp_active,lift_active,lift_extra_mA,torque_active,torque_mA,iq_slew_active,iq_slew_limited_mA,q_ff_mV,speed_boost_mA
```

## 字段用途

速度跟随：

```text
ref_rpm
ctrl_ref_rpm
fdb_rpm
ctrl_fdb_rpm
err_rpm
ramp_active
```

用于判断参考速度是否已经给到、速度斜坡是否太慢、反馈速度是否卡在低速。

扭矩和限流：

```text
iq_ref_mA
iq_mA
current_peak_mA
lift_active
lift_extra_mA
torque_active
torque_mA
iq_slew_active
iq_slew_limited_mA
speed_boost_mA
```

用于判断是速度环没有给足 q 轴电流、限流不够、还是 q 轴电流斜率限制导致起步慢。

电压和占空比：

```text
vq_mV
vbus_mV
duty_a
duty_b
duty_c
q_ff_mV
```

用于判断是否已经接近电压或占空比能力上限。

Hall 和角度：

```text
hall_raw
hall_sector
sector_no_change_count
edge_elapsed_us
theta_hall
theta_pred
theta_ctrl
```

用于判断是否长时间没有 Hall 边沿、角度预测是否和 Hall 角度偏离，或者是否出现卡齿/堵转趋势。

状态：

```text
direction
state
fault
ctrl_source
```

用于确认当前方向、是否仍在 running、是否发生故障、是否为速度模式。

## 发送数据建议

优先发送完整 `0 .. g_foc_start_log_idx - 1` 数据。
如果 Watch 导出太麻烦，先发送前 120 行和最后 120 行。

同时说明测试条件：

```text
motor_id:
方向: 抬升/下降
目标速度:
是否带载:
按键持续时间:
主观现象: 拉锯/慢起/机械声/突然上速
```
