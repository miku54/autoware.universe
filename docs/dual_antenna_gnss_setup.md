# 双天线GNSS定位配置指南

## 概述

本指南说明如何在Autoware.universe中配置和使用双天线GNSS进行车辆定位。双天线GNSS系统可以提供高精度的航向角信息，结合GNSS位置数据实现更准确的定位。

## 系统架构

```
双天线GNSS接收器
├── 位置数据 (NavSatFix) → /fix
└── 航向数据 (GnssInsOrientationStamped) → /autoware_orientation
                                           ↓
                                    GNSS Poser
                                           ↓
                                    位姿数据 (PoseWithCovarianceStamped)
                                           ↓
                                    EKF定位器
                                           ↓
                                    最终定位结果
```

## 配置步骤

### 1. 硬件要求

- 双天线GNSS接收器
- 支持RTK的GNSS模块
- 能够输出航向角信息的GNSS设备

### 2. 消息格式

#### 位置数据 (NavSatFix)
```yaml
header:
  stamp: {secs: 0, nsecs: 0}
  frame_id: "gnss_antenna"
latitude: 35.123456
longitude: 139.123456
altitude: 10.0
position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
position_covariance_type: 1
status:
  status: 0
  service: 1
```

#### 航向数据 (GnssInsOrientationStamped)
```yaml
header:
  stamp: {secs: 0, nsecs: 0}
  frame_id: "gnss_antenna"
orientation:
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
  rmse_rotation_x: 0.1
  rmse_rotation_y: 0.1
  rmse_rotation_z: 0.05
```

### 3. 启动系统

#### 启动传感器和GNSS定位
```bash
ros2 launch dual_antenna_gnss_sensing.launch.xml
```

#### 启动完整定位系统
```bash
ros2 launch dual_antenna_gnss_localization.launch.xml
```

### 4. 参数配置

#### GNSS Poser参数
```yaml
use_gnss_ins_orientation: true  # 启用双天线航向
gnss_pose_pub_method: 0         # 0: 即时值, 1: 平均值, 2: 中值
buff_epoch: 1                   # 位置缓冲大小
```

#### EKF定位器参数
```yaml
pose_measurement:
  pose_gate_dist: 49.5          # 位姿门限距离
  pose_measure_uncertainty_time: 0.01
process_noise:
  proc_stddev_yaw_c: 0.005      # 航向过程噪声
  proc_stddev_vx_c: 10.0        # 速度过程噪声
```

## 话题映射

### 输入话题
- `/fix` - GNSS位置数据
- `/autoware_orientation` - 双天线航向数据
- `/sensing/imu/imu_data` - IMU数据
- `/sensing/lidar/concatenated/pointcloud` - 激光雷达点云

### 输出话题
- `/localization/gnss/pose` - GNSS位姿
- `/localization/gnss/pose_with_covariance` - 带协方差的GNSS位姿
- `/localization/pose_estimator/pose` - 最终定位位姿
- `/localization/pose_estimator/pose_with_covariance` - 带协方差的最终位姿

## 故障排除

### 1. GNSS信号问题
- 检查天线连接
- 确保RTK基站信号正常
- 验证GNSS接收器配置

### 2. 航向数据问题
- 检查双天线安装距离（建议1-2米）
- 验证航向数据发布频率
- 确认消息格式正确

### 3. 定位精度问题
- 调整EKF参数
- 检查传感器标定
- 验证坐标系转换

## 性能优化

1. **提高更新频率**：确保GNSS数据更新频率≥10Hz
2. **优化协方差**：根据实际精度调整协方差矩阵
3. **多传感器融合**：结合激光雷达和IMU数据提高精度
4. **RTK增强**：使用RTK基站提高定位精度

## 注意事项

1. 双天线安装需要保持水平，距离建议1-2米
2. 航向角精度与天线间距成正比
3. 在隧道或高架桥下可能失去GNSS信号
4. 需要定期校准传感器坐标系