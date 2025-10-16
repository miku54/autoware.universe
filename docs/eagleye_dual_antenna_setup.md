# Eagleye双天线GNSS定位配置指南

## 概述

Eagleye是一个专门用于GNSS/INS融合的定位算法，特别适合处理双天线GNSS数据。本指南说明如何在Autoware.universe中配置和使用Eagleye进行双天线GNSS定位调试。

## 系统架构

```
双天线GNSS接收器
├── 位置数据 (NavSatFix) → /fix
└── 航向数据 (GnssInsOrientationStamped) → /autoware_orientation
                                           ↓
                                    Eagleye算法
                                           ↓
                                    位姿数据 (PoseWithCovarianceStamped)
                                           ↓
                                    最终定位结果
```

## Eagleye算法特点

### 1. 多传感器融合
- **GNSS位置**：高精度位置信息
- **双天线航向**：高精度航向角信息
- **IMU数据**：角速度和加速度信息
- **速度数据**：车辆速度信息

### 2. 核心算法模块
- **航向估计**：基于双天线GNSS的航向角计算
- **位置估计**：GNSS/INS融合位置估计
- **速度估计**：多传感器速度融合
- **轨迹平滑**：轨迹数据平滑处理
- **质量控制**：数据质量监控和异常检测

## 配置步骤

### 1. 硬件要求

- 双天线GNSS接收器
- 支持RTK的GNSS模块
- 高精度IMU传感器
- 车辆速度传感器

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

#### 启动Eagleye双天线GNSS定位
```bash
ros2 launch eagleye_dual_antenna_gnss.launch.xml
```

#### 启动调试模式
```bash
ros2 launch eagleye_debug.launch.xml
```

#### 启动测试模式
```bash
ros2 launch eagleye_test.launch.xml
```

### 4. 参数配置

#### 基本配置
```yaml
use_gnss_ins_orientation: true    # 启用双天线航向
use_multi_antenna_mode: true      # 启用多天线模式
use_rtk_heading: true             # 启用RTK航向
use_rtk_dead_reckoning: false     # 禁用RTK航位推算
```

#### GNSS配置
```yaml
gnss:
  dual_antenna_mode: true         # 双天线模式
  antenna_baseline: 1.0           # 天线间距 (米)
  heading_accuracy_threshold: 0.5 # 航向角精度阈值 (度)
  rtk_status_required: true       # RTK状态要求
  position_accuracy_threshold: 1.0 # 位置精度阈值 (米)
```

#### 航向配置
```yaml
heading:
  estimation_method: "dual_antenna"  # 双天线航向估计
  smoothing: true                    # 航向角平滑
  interpolation: true                # 航向角插值
  accuracy_threshold: 0.1            # 航向角精度阈值 (度)
  baseline_length: 1.0               # 双天线基线长度 (米)
```

## 话题映射

### 输入话题
- `/fix` - GNSS位置数据
- `/autoware_orientation` - 双天线航向数据
- `/sensing/imu/imu_data` - IMU数据
- `/localization/gyro_odometer/twist_with_covariance` - 速度数据

### 输出话题
- `/localization/pose_estimator/eagleye/pose_with_covariance` - Eagleye位姿
- `/localization/twist_estimator/eagleye/twist_with_covariance` - Eagleye速度
- `/localization/eagleye/geo_pose_with_covariance` - 地理位姿

## 调试工具

### 1. 调试脚本
```bash
python3 scripts/eagleye_debug.py
```

### 2. 可视化工具
- **RViz2**：位姿和轨迹可视化
- **rqt_topic**：话题监控
- **rqt_plot**：数据绘图
- **rqt_graph**：节点图监控

### 3. 数据记录
```bash
ros2 bag record -o eagleye_debug /fix /autoware_orientation /sensing/imu/imu_data /localization/pose_estimator/eagleye/pose_with_covariance
```

## 性能优化

### 1. 参数调优
- **航向精度**：调整 `heading_accuracy_threshold`
- **位置精度**：调整 `position_accuracy_threshold`
- **融合权重**：调整各传感器融合权重
- **滤波器参数**：调整卡尔曼滤波器参数

### 2. 硬件优化
- **天线安装**：确保双天线水平安装，间距1-2米
- **IMU标定**：定期校准IMU传感器
- **GNSS配置**：优化GNSS接收器参数

### 3. 软件优化
- **更新频率**：提高传感器数据更新频率
- **数据质量**：实施数据质量控制
- **异常检测**：启用异常值检测和处理

## 故障排除

### 1. 常见问题

#### 航向角不准确
- 检查双天线安装距离
- 验证航向数据质量
- 调整航向角精度阈值

#### 位置漂移
- 检查GNSS信号质量
- 验证RTK状态
- 调整位置精度阈值

#### 融合效果差
- 检查传感器标定
- 调整融合权重
- 验证数据同步

### 2. 调试步骤

1. **检查传感器状态**
   ```bash
   ros2 topic echo /fix
   ros2 topic echo /autoware_orientation
   ros2 topic echo /sensing/imu/imu_data
   ```

2. **监控Eagleye输出**
   ```bash
   ros2 topic echo /localization/pose_estimator/eagleye/pose_with_covariance
   ```

3. **查看调试信息**
   ```bash
   ros2 run eagleye_rt eagleye_debug.py
   ```

4. **分析性能数据**
   - 查看保存的调试数据
   - 分析轨迹精度
   - 检查融合效果

## 最佳实践

### 1. 系统配置
- 使用高质量的GNSS接收器
- 确保双天线正确安装
- 定期校准传感器

### 2. 参数设置
- 根据实际环境调整参数
- 定期验证定位精度
- 监控系统性能

### 3. 调试方法
- 使用调试工具监控系统状态
- 记录和分析调试数据
- 根据结果调整参数

## 注意事项

1. **天线安装**：双天线需要保持水平，距离建议1-2米
2. **数据同步**：确保所有传感器数据时间同步
3. **环境条件**：在开阔环境中测试效果最佳
4. **标定要求**：定期校准传感器坐标系
5. **性能监控**：持续监控定位精度和系统稳定性