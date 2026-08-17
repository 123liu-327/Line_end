# 普通巡线转弯响应 V2 备用参数

当前正式启用的是 `follow_test.launch` 中的响应增强 V1。V1 保持：

```xml
<arg name="aim_distance" default="0.10" />
```

不变，通过提高角速度响应并限制弯道线速度来减少压线。

## 可直接使用的 launch 备份

- 修改前原始版：`MD/follow_test_control_original_backup.launch`
- V2 激进备用版：`MD/follow_test_control_responsive_v2_backup.launch`

这两个文件都是约 349 行的完整 `follow_test.launch` 副本，不依赖 include 正式
文件。可以直接整文件复制覆盖，也可以单独启动进行对比测试。

恢复修改前原始版：

```bash
cp /home/ucar/instant_ws/src/flow_end/MD/follow_test_control_original_backup.launch \
   /home/ucar/instant_ws/src/flow_end/launch/follow_test.launch
```

切换为 V2 激进版：

```bash
cp /home/ucar/instant_ws/src/flow_end/MD/follow_test_control_responsive_v2_backup.launch \
   /home/ucar/instant_ws/src/flow_end/launch/follow_test.launch
```

也可以不覆盖正式文件，直接启动原始版：

```bash
roslaunch /home/ucar/instant_ws/src/flow_end/MD/follow_test_control_original_backup.launch
```

或者直接启动 V2：

```bash
roslaunch /home/ucar/instant_ws/src/flow_end/MD/follow_test_control_responsive_v2_backup.launch
```

两个完整备份都明确保留 `aim_distance=0.10m`，并包含制作备份时其余最新的
相机、预转、停车和Y岔路参数。

## 修改前原始参数（完整回退备份）

以下是本次增强转弯响应之前，`follow_test.launch` 使用的原始参数。需要完全
回退时可直接按此区块恢复。原始前视距离和基础线速度为：

```xml
<arg name="base_speed" default="0.30" />
<arg name="aim_distance" default="0.10" />
<arg name="aim_y_bias_m" default="0.20" />
```

原始运动控制参数为：

```xml
<arg name="control_path_smooth_window" default="2" />
<arg name="control_path_ema_alpha" default="0.35" />
<arg name="control_error_filter_alpha" default="0.65" />
<arg name="control_yaw_deadband" default="0.015" />
<arg name="control_kp_yaw" default="1.00" />
<arg name="control_ki_yaw" default="0.00" />
<arg name="control_kd_yaw" default="0.08" />
<arg name="control_integral_limit" default="0.30" />
<arg name="control_integral_error_threshold" default="0.25" />
<arg name="control_adaptive_error_threshold" default="0.35" />
<arg name="control_adaptive_kp_scale" default="1.25" />
<arg name="control_adaptive_kd_scale" default="1.40" />
<arg name="control_max_wz" default="0.65" />
<arg name="control_soft_wz_limit" default="0.55" />
<arg name="control_max_wz_rate" default="1.80" />
<arg name="control_turn_slowdown" default="0.65" />
<arg name="control_slow_error" default="0.45" />
<arg name="control_min_speed" default="0.05" />
<arg name="control_degraded_speed_scale" default="0.75" />
<arg name="control_max_accel" default="0.35" />
<arg name="control_max_decel" default="0.80" />
<arg name="control_cmd_filter_alpha" default="0.45" />
<arg name="lost_line_coast_sec" default="0.15" />
<arg name="lost_line_coast_speed_scale" default="0.60" />
```

## V2 激进版（当前未启用）

只有在 V1 实车测试仍表现为入弯偏慢、持续压线时，才将
`flow_end/launch/follow_test.launch` 中对应参数替换为以下数值：

```xml
<arg name="control_path_smooth_window" default="2" />
<arg name="control_path_ema_alpha" default="0.65" />
<arg name="control_error_filter_alpha" default="0.82" />
<arg name="control_yaw_deadband" default="0.010" />
<arg name="control_kp_yaw" default="1.55" />
<arg name="control_ki_yaw" default="0.00" />
<arg name="control_kd_yaw" default="0.10" />
<arg name="control_integral_limit" default="0.30" />
<arg name="control_integral_error_threshold" default="0.25" />
<arg name="control_adaptive_error_threshold" default="0.20" />
<arg name="control_adaptive_kp_scale" default="1.45" />
<arg name="control_adaptive_kd_scale" default="1.50" />
<arg name="control_max_wz" default="0.95" />
<arg name="control_soft_wz_limit" default="0.88" />
<arg name="control_max_wz_rate" default="3.40" />
<arg name="control_turn_slowdown" default="0.88" />
<arg name="control_slow_error" default="0.25" />
<arg name="control_min_speed" default="0.05" />
<arg name="control_degraded_speed_scale" default="0.75" />
<arg name="control_max_accel" default="0.35" />
<arg name="control_max_decel" default="1.60" />
<arg name="control_cmd_filter_alpha" default="0.72" />
```

## 两个版本的主要区别

| 参数 | V1（当前启用） | V2（备用） |
|---|---:|---:|
| `control_kp_yaw` | 1.30 | 1.55 |
| `control_max_wz` | 0.80 rad/s | 0.95 rad/s |
| `control_soft_wz_limit` | 0.72 rad/s | 0.88 rad/s |
| `control_max_wz_rate` | 2.60 rad/s² | 3.40 rad/s² |
| `control_turn_slowdown` | 0.80 | 0.88 |
| `control_slow_error` | 0.30 rad | 0.25 rad |

V2 转向更快、弯道降速更强，但出现左右摆动或出弯过冲的风险也更高。
切换后需要重新启动 `follow_test`；这里只修改 launch 参数，不需要重新编译。
