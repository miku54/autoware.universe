#!/usr/bin/env python3
"""
Eagleye双天线GNSS调试脚本
用于监控和调试Eagleye定位系统的性能
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import json
import os

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from std_msgs.msg import String

class EagleyeDebugger(Node):
    def __init__(self):
        super().__init__('eagleye_debugger')
        
        # 配置参数
        self.declare_parameter('debug_data_path', '/tmp/eagleye_debug')
        self.declare_parameter('save_frequency', 1.0)
        self.declare_parameter('plot_frequency', 0.1)
        
        self.debug_data_path = self.get_parameter('debug_data_path').value
        self.save_frequency = self.get_parameter('save_frequency').value
        self.plot_frequency = self.get_parameter('plot_frequency').value
        
        # 创建调试数据目录
        os.makedirs(self.debug_data_path, exist_ok=True)
        
        # 数据存储
        self.data = {
            'timestamps': [],
            'gnss_position': {'lat': [], 'lon': [], 'alt': []},
            'gnss_heading': {'x': [], 'y': [], 'z': [], 'w': []},
            'eagleye_pose': {'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []},
            'eagleye_twist': {'vx': [], 'vy': [], 'vz': [], 'wx': [], 'wy': [], 'wz': []},
            'imu_data': {'ax': [], 'ay': [], 'az': [], 'gx': [], 'gy': [], 'gz': []},
            'covariance': {'position': [], 'orientation': []}
        }
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 订阅话题
        self.gnss_fix_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gnss_fix_callback,
            qos_profile
        )
        
        self.gnss_orientation_sub = self.create_subscription(
            GnssInsOrientationStamped,
            '/autoware_orientation',
            self.gnss_orientation_callback,
            qos_profile
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/sensing/imu/imu_data',
            self.imu_callback,
            qos_profile
        )
        
        self.eagleye_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/eagleye/pose_with_covariance',
            self.eagleye_pose_callback,
            qos_profile
        )
        
        self.eagleye_twist_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            '/localization/twist_estimator/eagleye/twist_with_covariance',
            self.eagleye_twist_callback,
            qos_profile
        )
        
        # 定时器
        self.save_timer = self.create_timer(1.0 / self.save_frequency, self.save_data)
        self.plot_timer = self.create_timer(1.0 / self.plot_frequency, self.update_plots)
        
        # 状态监控
        self.status = {
            'gnss_fix_received': False,
            'gnss_orientation_received': False,
            'imu_received': False,
            'eagleye_pose_received': False,
            'eagleye_twist_received': False,
            'last_update_time': None
        }
        
        self.get_logger().info('Eagleye调试器已启动')
    
    def gnss_fix_callback(self, msg):
        """GNSS位置数据回调"""
        self.status['gnss_fix_received'] = True
        self.status['last_update_time'] = self.get_clock().now()
        
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.data['timestamps'].append(timestamp)
        self.data['gnss_position']['lat'].append(msg.latitude)
        self.data['gnss_position']['lon'].append(msg.longitude)
        self.data['gnss_position']['alt'].append(msg.altitude)
        
        self.get_logger().debug(f'GNSS位置: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}')
    
    def gnss_orientation_callback(self, msg):
        """GNSS航向数据回调"""
        self.status['gnss_orientation_received'] = True
        
        self.data['gnss_heading']['x'].append(msg.orientation.orientation.x)
        self.data['gnss_heading']['y'].append(msg.orientation.orientation.y)
        self.data['gnss_heading']['z'].append(msg.orientation.orientation.z)
        self.data['gnss_heading']['w'].append(msg.orientation.orientation.w)
        
        # 计算航向角
        heading = self.quaternion_to_heading(
            msg.orientation.orientation.x,
            msg.orientation.orientation.y,
            msg.orientation.orientation.z,
            msg.orientation.orientation.w
        )
        
        self.get_logger().debug(f'GNSS航向: {heading:.2f}°')
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        self.status['imu_received'] = True
        
        self.data['imu_data']['ax'].append(msg.linear_acceleration.x)
        self.data['imu_data']['ay'].append(msg.linear_acceleration.y)
        self.data['imu_data']['az'].append(msg.linear_acceleration.z)
        self.data['imu_data']['gx'].append(msg.angular_velocity.x)
        self.data['imu_data']['gy'].append(msg.angular_velocity.y)
        self.data['imu_data']['gz'].append(msg.angular_velocity.z)
    
    def eagleye_pose_callback(self, msg):
        """Eagleye位姿数据回调"""
        self.status['eagleye_pose_received'] = True
        
        self.data['eagleye_pose']['x'].append(msg.pose.pose.position.x)
        self.data['eagleye_pose']['y'].append(msg.pose.pose.position.y)
        self.data['eagleye_pose']['z'].append(msg.pose.pose.position.z)
        self.data['eagleye_pose']['qx'].append(msg.pose.pose.orientation.x)
        self.data['eagleye_pose']['qy'].append(msg.pose.pose.orientation.y)
        self.data['eagleye_pose']['qz'].append(msg.pose.pose.orientation.z)
        self.data['eagleye_pose']['qw'].append(msg.pose.pose.orientation.w)
        
        # 存储协方差
        cov = msg.pose.covariance
        self.data['covariance']['position'].append([cov[0], cov[7], cov[14]])
        self.data['covariance']['orientation'].append([cov[21], cov[28], cov[35]])
    
    def eagleye_twist_callback(self, msg):
        """Eagleye速度数据回调"""
        self.status['eagleye_twist_received'] = True
        
        self.data['eagleye_twist']['vx'].append(msg.twist.twist.linear.x)
        self.data['eagleye_twist']['vy'].append(msg.twist.twist.linear.y)
        self.data['eagleye_twist']['vz'].append(msg.twist.twist.linear.z)
        self.data['eagleye_twist']['wx'].append(msg.twist.twist.angular.x)
        self.data['eagleye_twist']['wy'].append(msg.twist.twist.angular.y)
        self.data['eagleye_twist']['wz'].append(msg.twist.twist.angular.z)
    
    def quaternion_to_heading(self, x, y, z, w):
        """四元数转航向角"""
        # 计算航向角 (yaw)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return np.degrees(yaw)
    
    def save_data(self):
        """保存调试数据"""
        if len(self.data['timestamps']) == 0:
            return
        
        # 保存为JSON格式
        json_file = os.path.join(self.debug_data_path, f'eagleye_debug_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
        with open(json_file, 'w') as f:
            json.dump(self.data, f, indent=2)
        
        # 保存状态信息
        status_file = os.path.join(self.debug_data_path, 'eagleye_status.json')
        with open(status_file, 'w') as f:
            json.dump(self.status, f, indent=2)
        
        self.get_logger().info(f'调试数据已保存到: {json_file}')
    
    def update_plots(self):
        """更新图表"""
        if len(self.data['timestamps']) < 2:
            return
        
        try:
            # 创建位置轨迹图
            plt.figure(figsize=(12, 8))
            
            # 子图1: 位置轨迹
            plt.subplot(2, 2, 1)
            if len(self.data['eagleye_pose']['x']) > 0:
                plt.plot(self.data['eagleye_pose']['x'], self.data['eagleye_pose']['y'], 'b-', label='Eagleye轨迹')
            if len(self.data['gnss_position']['lat']) > 0:
                # 简单的经纬度转换 (仅用于显示)
                lat_offset = self.data['gnss_position']['lat'][0] if self.data['gnss_position']['lat'] else 0
                lon_offset = self.data['gnss_position']['lon'][0] if self.data['gnss_position']['lon'] else 0
                x_gnss = [(lon - lon_offset) * 111320 * np.cos(np.radians(lat_offset)) for lon in self.data['gnss_position']['lon']]
                y_gnss = [(lat - lat_offset) * 111320 for lat in self.data['gnss_position']['lat']]
                plt.plot(x_gnss, y_gnss, 'r--', label='GNSS轨迹')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title('位置轨迹')
            plt.legend()
            plt.grid(True)
            
            # 子图2: 航向角
            plt.subplot(2, 2, 2)
            if len(self.data['gnss_heading']['x']) > 0:
                headings = [self.quaternion_to_heading(x, y, z, w) for x, y, z, w in 
                           zip(self.data['gnss_heading']['x'], self.data['gnss_heading']['y'], 
                               self.data['gnss_heading']['z'], self.data['gnss_heading']['w'])]
                plt.plot(headings, 'g-', label='GNSS航向')
            plt.xlabel('时间步')
            plt.ylabel('航向角 (度)')
            plt.title('航向角变化')
            plt.legend()
            plt.grid(True)
            
            # 子图3: 速度
            plt.subplot(2, 2, 3)
            if len(self.data['eagleye_twist']['vx']) > 0:
                plt.plot(self.data['eagleye_twist']['vx'], 'b-', label='Vx')
                plt.plot(self.data['eagleye_twist']['vy'], 'g-', label='Vy')
            plt.xlabel('时间步')
            plt.ylabel('速度 (m/s)')
            plt.title('速度变化')
            plt.legend()
            plt.grid(True)
            
            # 子图4: 状态监控
            plt.subplot(2, 2, 4)
            status_names = list(self.status.keys())[:-1]  # 排除last_update_time
            status_values = [1 if self.status[key] else 0 for key in status_names]
            plt.bar(status_names, status_values)
            plt.title('传感器状态')
            plt.xticks(rotation=45)
            plt.ylim(0, 1)
            
            plt.tight_layout()
            plt.savefig(os.path.join(self.debug_data_path, 'eagleye_debug_plot.png'))
            plt.close()
            
        except Exception as e:
            self.get_logger().error(f'绘图错误: {str(e)}')
    
    def print_status(self):
        """打印状态信息"""
        self.get_logger().info('=== Eagleye调试状态 ===')
        for key, value in self.status.items():
            if key != 'last_update_time':
                status_str = '✓' if value else '✗'
                self.get_logger().info(f'{key}: {status_str}')
        
        if self.status['last_update_time']:
            elapsed = (self.get_clock().now() - self.status['last_update_time']).nanoseconds / 1e9
            self.get_logger().info(f'最后更新: {elapsed:.2f}秒前')
        
        self.get_logger().info(f'数据点数量: {len(self.data["timestamps"])}')

def main(args=None):
    rclpy.init(args=args)
    
    debugger = EagleyeDebugger()
    
    try:
        # 创建定时器打印状态
        def print_status():
            debugger.print_status()
        
        status_timer = debugger.create_timer(5.0, print_status)
        
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        debugger.get_logger().info('调试器停止')
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()