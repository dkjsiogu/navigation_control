#!/usr/bin/env python3
"""
路径跟踪控制器 (Pure Pursuit for Omnidirectional Robot)
功能:
1. 订阅 A* 规划的路径 (/planned_path)
2. 使用 Pure Pursuit 算法跟踪路径
3. 全向轮控制：保持朝向 + vx/vy 移动
4. 适配 8Hz 雷达，不旋转机器人

作者: SLAM 大师们 🔥
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class SimpleGoalController(Node):
    def __init__(self):
        super().__init__('path_tracker')
        
        # 参数 (🔧 8Hz雷达优化)
        self.declare_parameter('max_linear_vel', 0.3)     # 8Hz雷达最优巡航速度
        self.declare_parameter('max_angular_vel', 0.5)    # 约30°/s，避免IMU积分误差
        self.declare_parameter('goal_tolerance', 0.05)    # 🔧 降低到5cm - 更精确到达
        self.declare_parameter('yaw_tolerance', 0.052)    # 朝向容差 ~3度 (更精确)
        self.declare_parameter('lookahead_distance', 0.5) # Pure Pursuit 前瞻距离
        self.declare_parameter('waypoint_tolerance', 0.15) # 路径点切换容差
        self.declare_parameter('position_hold_time', 1.0)  # 🔧 新增:位置保持时间(秒)
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.position_hold_time = self.get_parameter('position_hold_time').value
        
        # 订阅规划路径
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_status_pub = self.create_publisher(String, '/goal_status', 10)
        
        # TF监听器 - 用于获取 map -> base_link 变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时器 - 控制循环 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 路径跟踪状态
        self.current_path = None      # 当前路径 (list of PoseStamped)
        self.current_waypoint_idx = 0 # 当前目标路径点索引
        self.initial_yaw = None       # 出发时的朝向
        self.goal_yaw = None          # 目标朝向
        self.position_reached = False # 位置是否到达
        self.position_stable_start_time = None  # 🔧 新增:位置稳定开始时间
        
        self.get_logger().info('🚀 路径跟踪控制器已启动')
        self.get_logger().info(f'最大速度: {self.max_linear_vel} m/s')
        self.get_logger().info(f'前瞻距离: {self.lookahead_distance} m')
        self.get_logger().info(f'目标容差: {self.goal_tolerance} m')
        self.get_logger().info('策略: Pure Pursuit + 全向轮 + 保持朝向')
    
    def path_callback(self, msg):
        """接收规划好的路径"""
        if len(msg.poses) < 2:
            self.get_logger().warn('路径太短，忽略')
            return
        
        self.current_path = msg.poses
        self.current_waypoint_idx = 0
        self.position_reached = False
        
        # 提取目标朝向（从最后一个路径点）
        goal_pose = msg.poses[-1].pose.orientation
        self.goal_yaw = math.atan2(
            2 * (goal_pose.w * goal_pose.z + goal_pose.x * goal_pose.y),
            1 - 2 * (goal_pose.y * goal_pose.y + goal_pose.z * goal_pose.z)
        )
        
        # 记录出发时的朝向
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.initial_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            start = msg.poses[0].pose.position
            goal = msg.poses[-1].pose.position
            
            self.get_logger().info(f'✅ 收到新路径: {len(msg.poses)} 个路径点')
            self.get_logger().info(f'   起点: ({start.x:.2f}, {start.y:.2f})')
            self.get_logger().info(f'   终点: ({goal.x:.2f}, {goal.y:.2f})')
            self.get_logger().info(f'🧭 锁定初始朝向: {math.degrees(self.initial_yaw):.1f}°')
            self.get_logger().info(f'🎯 目标朝向: {math.degrees(self.goal_yaw):.1f}°')
            self.get_logger().info('🚗 开始跟踪路径...')
            
        except TransformException as ex:
            self.get_logger().error(f'无法获取当前位姿: {ex}')
            self.current_path = None
    
    def control_loop(self):
        """控制循环 - Pure Pursuit 路径跟踪"""
        if self.current_path is None or len(self.current_path) == 0:
            # 没有路径时发布零速度
            self.publish_velocity(0.0, 0.0, 0.0)
            return
        
        # 获取机器人当前位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except TransformException as ex:
            self.get_logger().warn(f'无法获取 TF: {ex}', throttle_duration_sec=1.0)
            return
        
        # 找到当前应该追踪的路径点
        target_waypoint = self.find_lookahead_point(current_x, current_y)
        
        if target_waypoint is None:
            # 路径追踪完成，直接奔向终点
            target_x = self.current_path[-1].pose.position.x
            target_y = self.current_path[-1].pose.position.y
        else:
            target_x = target_waypoint.pose.position.x
            target_y = target_waypoint.pose.position.y
        
        # 计算到目标路径点的向量 (map坐标系)
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 检查是否到达终点位置
        goal_x = self.current_path[-1].pose.position.x
        goal_y = self.current_path[-1].pose.position.y
        dist_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        
        # 阶段1：到达位置并保持稳定
        if not self.position_reached:
            if dist_to_goal < self.goal_tolerance:
                # 进入目标范围,开始计时
                if self.position_stable_start_time is None:
                    self.position_stable_start_time = self.get_clock().now()
                    self.get_logger().info(f'📍 进入目标范围({dist_to_goal*100:.1f}cm),开始稳定保持...')
                
                # 检查是否已稳定保持足够时间
                stable_duration = (self.get_clock().now() - self.position_stable_start_time).nanoseconds / 1e9
                if stable_duration >= self.position_hold_time:
                    self.get_logger().info(f'✅ 位置稳定保持{stable_duration:.1f}秒,确认到达!')
                    self.position_reached = True
                    self.position_stable_start_time = None
                    self.publish_velocity(0.0, 0.0, 0.0)
                    return
                
                # 持续微调保持位置(即使在容差内也继续修正)
                # 这样可以对抗打滑/漂移
                speed = max(dist_to_goal * 2.0, 0.05)  # 最小5cm/s持续修正
            else:
                # 离开目标范围,重置计时
                self.position_stable_start_time = None
                
                # 正常速度规划 (8Hz雷达优化)
                if dist_to_goal > 1.0:
                    speed = self.max_linear_vel  # 远距离：0.3m/s巡航
                elif dist_to_goal > 0.5:
                    speed = 0.25  # 中距离：稳定减速避免突变
                elif dist_to_goal > 0.2:
                    speed = 0.15  # 近距离：低速精准接近
                else:
                    speed = max(dist_to_goal * 0.6, 0.08)  # 极近距离：缓慢收敛
            
            # 世界坐标系 → 机器人坐标系转换
            cos_yaw = math.cos(robot_yaw)
            sin_yaw = math.sin(robot_yaw)
            dx_robot = dx * cos_yaw + dy * sin_yaw   # 机器人X轴（前+）
            dy_robot = -dx * sin_yaw + dy * cos_yaw  # 机器人Y轴（左+）
            
            # 归一化方向并计算速度
            if distance > 0.01:
                vx = (dx_robot / distance) * speed
                vy = (dy_robot / distance) * speed
            else:
                vx = 0.0
                vy = 0.0
            
            # 保持初始朝向（不旋转，只纠正漂移）
            wz = 0.0
            if self.initial_yaw is not None:
                yaw_error = self.normalize_angle(self.initial_yaw - robot_yaw)
                if abs(yaw_error) > 0.05:  # 死区 > 2.9°
                    # 8Hz雷达：降低增益避免震荡
                    wz = max(-self.max_angular_vel * 0.8, min(self.max_angular_vel * 0.8, 1.5 * yaw_error))
            
            self.publish_velocity(vx, vy, wz)
            
            # 日志输出
            self.get_logger().info(
                f'路径点 {self.current_waypoint_idx}/{len(self.current_path)} | '
                f'距终点: {dist_to_goal:.2f}m | '
                f'速度: vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}',
                throttle_duration_sec=0.5
            )
        
        # 阶段2：位置到达后，旋转到目标朝向
        else:
            if self.goal_yaw is None:
                # 没有目标朝向，直接完成
                self.get_logger().info('✅ 任务完成（无朝向要求）')
                self.publish_velocity(0.0, 0.0, 0.0)
                status_msg = String()
                status_msg.data = 'reached'
                self.goal_status_pub.publish(status_msg)
                self.current_path = None
                return
            
            # 计算朝向误差
            yaw_error = self.normalize_angle(self.goal_yaw - robot_yaw)
            
            if abs(yaw_error) < self.yaw_tolerance:
                # 朝向也到达了，任务完成
                self.get_logger().info(f'🎯 到达目标朝向！误差: {math.degrees(yaw_error):.2f}°')
                self.get_logger().info('✅ 任务完成（位置+朝向）')
                self.publish_velocity(0.0, 0.0, 0.0)
                
                status_msg = String()
                status_msg.data = 'reached'
                self.goal_status_pub.publish(status_msg)
                
                self.current_path = None
                return
            
            # 继续旋转 - 三段控制 + 预判停止
            # 远(>15°)→中(5-15°)→近(<5°)
            abs_error = abs(yaw_error)
            
            if abs_error > 0.26:  # >15° 快速接近
                gain = 1.8
                speed_limit = 0.9
            elif abs_error > 0.087:  # 5-15° 减速
                gain = 1.2
                speed_limit = 0.6
            else:  # <5° 精细控制 + 预判停止
                # 当角速度×惯性时间 ≈ 剩余角度时提前停止
                # 假设0.2s惯性时间
                gain = 0.8
                speed_limit = 0.4
            
            wz = max(-self.max_angular_vel * speed_limit, 
                    min(self.max_angular_vel * speed_limit, gain * yaw_error))
            self.publish_velocity(0.0, 0.0, wz)
            
            self.get_logger().info(
                f'🔄 旋转到目标朝向 | '
                f'当前: {math.degrees(robot_yaw):.1f}° | '
                f'目标: {math.degrees(self.goal_yaw):.1f}° | '
                f'误差: {math.degrees(yaw_error):.1f}° | '
                f'角速度: {wz:.2f}',
                throttle_duration_sec=0.3
            )
    
    def find_lookahead_point(self, current_x, current_y):
        """找到前瞻距离内的目标路径点 (Pure Pursuit)"""
        # 从当前路径点开始，找到距离 > lookahead_distance 的点
        for i in range(self.current_waypoint_idx, len(self.current_path)):
            wp = self.current_path[i].pose
            dist = math.sqrt(
                (wp.position.x - current_x)**2 + 
                (wp.position.y - current_y)**2
            )
            
            # 如果已经通过这个路径点，切换到下一个
            if dist < self.waypoint_tolerance:
                self.current_waypoint_idx = min(i + 1, len(self.current_path) - 1)
                continue
            
            # 找到前瞻点
            if dist >= self.lookahead_distance:
                return self.current_path[i]
        
        # 如果没找到，返回最后一个点
        return None
    
    def publish_velocity(self, vx, vy, wz):
        """发布速度命令"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)
    
    def normalize_angle(self, angle):
        """归一化角度到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def interpolate_angle(self, angle1, angle2, t):
        """在两个角度之间插值
        angle1: 目标角度 (t=0时)
        angle2: 起始角度 (t=1时)
        t: 插值参数 [0, 1]
        返回: 插值后的角度
        """
        # 计算最短角度差
        diff = self.normalize_angle(angle1 - angle2)
        # 线性插值
        return self.normalize_angle(angle2 + diff * (1.0 - t))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
