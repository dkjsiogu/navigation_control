#!/usr/bin/env python3
"""
旋转控制器
==========
功能:
1. 控制机器人平滑旋转到目标朝向
2. 使用PID控制实现角速度控制
3. 支持最短路径旋转（顺时针/逆时针）
4. 防止角度突变和震荡

订阅:
- /odom (nav_msgs/Odometry): 机器人当前位姿
- /target_yaw (std_msgs/Float64): 目标朝向角（弧度）

发布:
- /cmd_vel (geometry_msgs/Twist): 速度指令
- /rotation_status (std_msgs/String): 旋转状态 (rotating/reached)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
import math

class RotationController(Node):
    def __init__(self):
        super().__init__('rotation_controller')
        
        # 参数
        self.declare_parameter('max_angular_velocity', 0.8)  # rad/s
        self.declare_parameter('min_angular_velocity', 0.1)  # rad/s
        self.declare_parameter('angular_tolerance', 0.05)    # ~2.9度
        self.declare_parameter('kp', 1.5)                    # P控制增益
        self.declare_parameter('ki', 0.0)                    # I控制增益
        self.declare_parameter('kd', 0.3)                    # D控制增益
        self.declare_parameter('control_frequency', 20.0)    # Hz
        
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.min_angular_vel = self.get_parameter('min_angular_velocity').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.control_freq = self.get_parameter('control_frequency').value
        
        # 状态
        self.current_yaw = 0.0
        self.target_yaw = None
        self.last_error = 0.0
        self.error_integral = 0.0
        self.is_rotating = False
        
        # 订阅
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.target_sub = self.create_subscription(
            Float64, '/target_yaw', self.target_yaw_callback, 10)
        
        # 发布
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/rotation_status', 10)
        
        # 控制定时器
        self.control_timer = self.create_timer(
            1.0 / self.control_freq, self.control_loop)
        
        self.get_logger().info('🔄 旋转控制器已启动')
        self.get_logger().info(f'   最大角速度: {self.max_angular_vel:.2f} rad/s')
        self.get_logger().info(f'   最小角速度: {self.min_angular_vel:.2f} rad/s')
        self.get_logger().info(f'   角度容差: {math.degrees(self.angular_tolerance):.1f}°')
        self.get_logger().info(f'   PID参数: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')
    
    def odom_callback(self, msg):
        """更新当前朝向"""
        quat = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
    
    def target_yaw_callback(self, msg):
        """接收目标朝向"""
        self.target_yaw = msg.data
        self.error_integral = 0.0  # 重置积分项
        self.is_rotating = True
        self.get_logger().info(f'🎯 目标朝向: {math.degrees(self.target_yaw):.1f}°')
    
    def normalize_angle(self, angle):
        """归一化角度到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def control_loop(self):
        """PID控制循环"""
        if self.target_yaw is None or not self.is_rotating:
            return
        
        # 计算角度误差（最短路径）
        error = self.normalize_angle(self.target_yaw - self.current_yaw)
        
        # 检查是否到达
        if abs(error) < self.angular_tolerance:
            # 停止旋转
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            # 发布到达状态
            status_msg = String()
            status_msg.data = 'reached'
            self.status_pub.publish(status_msg)
            
            self.get_logger().info(f'✅ 到达目标朝向 (误差: {math.degrees(error):.2f}°)')
            self.is_rotating = False
            self.target_yaw = None
            self.error_integral = 0.0
            self.last_error = 0.0
            return
        
        # PID计算
        dt = 1.0 / self.control_freq
        
        # P项
        p_term = self.kp * error
        
        # I项（带积分饱和限制）
        self.error_integral += error * dt
        max_integral = self.max_angular_vel / (self.ki + 1e-6)
        self.error_integral = max(-max_integral, min(max_integral, self.error_integral))
        i_term = self.ki * self.error_integral
        
        # D项
        error_derivative = (error - self.last_error) / dt
        d_term = self.kd * error_derivative
        
        # 计算控制量
        angular_vel = p_term + i_term + d_term
        
        # 限制角速度
        if abs(angular_vel) > self.max_angular_vel:
            angular_vel = math.copysign(self.max_angular_vel, angular_vel)
        elif abs(angular_vel) < self.min_angular_vel:
            angular_vel = math.copysign(self.min_angular_vel, angular_vel)
        
        # 发布速度指令
        cmd = Twist()
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
        
        # 发布旋转状态
        status_msg = String()
        status_msg.data = 'rotating'
        self.status_pub.publish(status_msg)
        
        # 更新上次误差
        self.last_error = error
        
        # 调试输出（降低频率）
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        if self._debug_counter % 10 == 0:  # 每0.5秒输出一次
            self.get_logger().debug(
                f'旋转中: 当前={math.degrees(self.current_yaw):.1f}° '
                f'目标={math.degrees(self.target_yaw):.1f}° '
                f'误差={math.degrees(error):.1f}° '
                f'角速度={angular_vel:.2f} rad/s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = RotationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
