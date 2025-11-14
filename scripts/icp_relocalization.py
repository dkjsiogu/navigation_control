#!/usr/bin/env python3
"""
基于 ICP 的自动重定位节点
使用激光扫描与地图进行 ICP 匹配，自动确定机器人位置
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
from scipy.spatial import KDTree
from scipy.optimize import minimize
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading


class ICPRelocalization(Node):
    def __init__(self):
        super().__init__('icp_relocalization')
        
        # 声明参数
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('max_iterations', 50)
        self.declare_parameter('convergence_threshold', 0.001)
        self.declare_parameter('max_correspondence_distance', 0.5)
        self.declare_parameter('min_scan_points', 50)
        self.declare_parameter('initial_x', 0.0)  # 搜索中心X坐标
        self.declare_parameter('initial_y', 0.0)  # 搜索中心Y坐标
        self.declare_parameter('search_grid_size', 2.0)  # 在±2m范围内搜索
        self.declare_parameter('search_grid_resolution', 0.5)
        self.declare_parameter('angle_search_range', 3.14159)  # ±180°
        self.declare_parameter('angle_search_step', 0.1745)  # 10°
        self.declare_parameter('auto_relocalize_interval', 5.0)  # 每5秒尝试一次
        
        # 获取参数
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.convergence_threshold = self.get_parameter('convergence_threshold').value
        self.max_corr_dist = self.get_parameter('max_correspondence_distance').value
        self.min_scan_points = self.get_parameter('min_scan_points').value
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.search_size = self.get_parameter('search_grid_size').value
        self.search_res = self.get_parameter('search_grid_resolution').value
        self.angle_range = self.get_parameter('angle_search_range').value
        self.angle_step = self.get_parameter('angle_search_step').value
        self.relocalize_interval = self.get_parameter('auto_relocalize_interval').value
        
        # 状态变量
        self.map_data = None
        self.map_origin = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_points = None  # 地图中的障碍物点
        self.latest_scan = None
        self.is_relocalized = False
        self.searched_poses_count = 0  # Add a counter for searched poses
        self.lock = threading.Lock()  # Lock for thread-safe operations
        self.map_received = False  # 标记是否已收到地图
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        
        # 发布初始位姿
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # 定时器 - 自动重定位
        self.relocalize_timer = self.create_timer(
            self.relocalize_interval,
            self.attempt_relocalization
        )
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🎯 ICP 自动重定位节点已启动')
        self.get_logger().info(f'   ICP 最大迭代: {self.max_iterations}')
        self.get_logger().info(f'   收敛阈值: {self.convergence_threshold:.4f}')
        self.get_logger().info(f'   搜索范围: ±{self.search_size}m, 角度±{math.degrees(self.angle_range):.0f}°')
        self.get_logger().info(f'   自动重定位间隔: {self.relocalize_interval}s')
        self.get_logger().info('=' * 70)
    
    def map_callback(self, msg: OccupancyGrid):
        """处理地图数据"""
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # 只在第一次收到地图时打印信息
        if not self.map_received:
            # 计算地图中心点（世界坐标系）
            map_center_x = self.map_origin[0] + (self.map_width * self.map_resolution) / 2.0
            map_center_y = self.map_origin[1] + (self.map_height * self.map_resolution) / 2.0
            
            # 搜索中心固定为(0, 0)
            self.get_logger().info(f'📍 搜索中心设置为: (0.000, 0.000)')
            self.map_received = True
        
        # 提取地图中的障碍物点
        map_array = np.array(msg.data).reshape((self.map_height, self.map_width))
        
        # 找到所有占据的格子 (值 > 50 认为是障碍物)
        occupied = np.where(map_array > 50)
        
        # 转换为世界坐标
        self.map_points = []
        for i in range(len(occupied[0])):
            y_grid = occupied[0][i]
            x_grid = occupied[1][i]
            x_world = self.map_origin[0] + x_grid * self.map_resolution
            y_world = self.map_origin[1] + y_grid * self.map_resolution
            self.map_points.append([x_world, y_world])
        
        self.map_points = np.array(self.map_points)
        

    
    def scan_callback(self, msg: LaserScan):
        """保存最新的激光扫描"""
        self.latest_scan = msg
    
    def scan_to_points(self, scan: LaserScan, pose=(0, 0, 0)):
        """将激光扫描转换为点云（相对于给定位姿）"""
        points = []
        x, y, theta = pose
        
        # URDF中激光雷达相对base_link的偏移和旋转
        # xyz="0.098 0.065 0.077" rpy="0 0 3.1416"
        # ⚠️ 实测闭环有顺时针2-3度偏差，补偿逆时针偏移
        laser_offset_x = 0.098+0.10  # 向前98mm + 补偿
        laser_offset_y = 0.065-0.04  # 向左65mm - 补偿
        laser_offset_angle = math.pi - 0.0349  # 180度 + 2度补偿 (2°=0.0349 rad)
        
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max or math.isnan(r) or math.isinf(r):
                continue
            
            # 1. 扫描点在激光雷达坐标系中的位置
            scan_angle = scan.angle_min + i * scan.angle_increment
            point_in_laser_x = r * math.cos(scan_angle)
            point_in_laser_y = r * math.sin(scan_angle)
            
            # 2. 转换到base_link坐标系（考虑180度旋转）
            cos_offset = math.cos(laser_offset_angle)
            sin_offset = math.sin(laser_offset_angle)
            point_in_base_x = laser_offset_x + point_in_laser_x * cos_offset - point_in_laser_y * sin_offset
            point_in_base_y = laser_offset_y + point_in_laser_x * sin_offset + point_in_laser_y * cos_offset
            
            # 3. 转换到世界坐标系（考虑base_link的位姿）
            cos_theta = math.cos(theta)
            sin_theta = math.sin(theta)
            px = x + point_in_base_x * cos_theta - point_in_base_y * sin_theta
            py = y + point_in_base_x * sin_theta + point_in_base_y * cos_theta
            
            points.append([px, py])
        
        return np.array(points)
    
    def icp_match(self, source_points, target_points, initial_pose=(0, 0, 0)):
        """
        ICP 匹配算法
        source_points: 激光扫描点云
        target_points: 地图点云
        initial_pose: 初始位姿 (x, y, theta)
        """
        if len(source_points) < 3 or len(target_points) < 3:
            return None, float('inf')
        
        # 构建 KD-Tree 加速最近邻搜索
        tree = KDTree(target_points)
        
        # 当前变换
        x, y, theta = initial_pose
        
        prev_error = float('inf')
        
        for iteration in range(self.max_iterations):
            # 1. 应用当前变换到源点云
            cos_theta = math.cos(theta)
            sin_theta = math.sin(theta)
            
            transformed = np.zeros_like(source_points)
            transformed[:, 0] = x + source_points[:, 0] * cos_theta - source_points[:, 1] * sin_theta
            transformed[:, 1] = y + source_points[:, 0] * sin_theta + source_points[:, 1] * cos_theta
            
            # 2. 找到最近邻对应点
            distances, indices = tree.query(transformed)
            
            # 过滤距离过大的对应
            valid = distances < self.max_corr_dist
            valid_count = np.sum(valid)
            
            if valid_count < max(10, len(source_points) * 0.3):  # 至少30%的点要匹配上
                break
            
            valid_source = transformed[valid]
            valid_target = target_points[indices[valid]]
            
            # 3. 计算均方误差
            error = np.mean(distances[valid] ** 2)
            
            # 检查收敛
            if abs(prev_error - error) < self.convergence_threshold:
                # 返回时附加匹配点数信息
                return (x, y, theta), error, valid_count
            
            prev_error = error
            
            # 4. 计算最优变换（使用质心对齐 + SVD）
            centroid_source = np.mean(valid_source, axis=0)
            centroid_target = np.mean(valid_target, axis=0)
            
            # 去中心化
            source_centered = valid_source - centroid_source
            target_centered = valid_target - centroid_target
            
            # 计算旋转矩阵 (SVD)
            H = source_centered.T @ target_centered
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # 确保旋转矩阵有效
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # 提取旋转角度
            new_theta = math.atan2(R[1, 0], R[0, 0])
            
            # 计算平移
            t = centroid_target - R @ centroid_source
            
            # 更新变换
            x += t[0]
            y += t[1]
            theta = new_theta
        
        # 返回最后的匹配结果和有效点数
        final_valid = np.sum(distances < self.max_corr_dist)
        return (x, y, theta), prev_error, final_valid
    
    def attempt_relocalization(self):
        """尝试自动重定位"""
        if self.is_relocalized:
            return
        
        if self.map_points is None or len(self.map_points) < 10:
            self.get_logger().warn('⚠️  地图未加载或点数不足，跳过重定位')
            return
        
        if self.latest_scan is None:
            self.get_logger().warn('⚠️  未收到激光扫描数据，跳过重定位')
            return
        
        self.get_logger().info('🔍 开始 ICP 自动重定位...')
        
        # 将激光扫描转换为点云（相对于原点）
        scan_points = self.scan_to_points(self.latest_scan)
        
        if len(scan_points) < self.min_scan_points:
            self.get_logger().warn(f'⚠️  扫描点数不足 ({len(scan_points)} < {self.min_scan_points})')
            return
        
        # 多起点搜索最佳匹配
        best_pose = None
        best_error = float('inf')
        best_valid_count = 0
        
        # 在搜索网格中尝试不同的初始位置和角度
        search_results = []
        x_range = np.arange(self.initial_x - self.search_size, self.initial_x + self.search_size + self.search_res, self.search_res)
        y_range = np.arange(self.initial_y - self.search_size, self.initial_y + self.search_size + self.search_res, self.search_res)
        angle_range = np.arange(-self.angle_range, self.angle_range + self.angle_step, self.angle_step)
        
        self.get_logger().info(f'   搜索中心: ({self.initial_x:.3f}, {self.initial_y:.3f})')
        self.get_logger().info(f'   网格设置: X {len(x_range)} × Y {len(y_range)} × 角度 {len(angle_range)}')
        
        # 直接生成网格（不用set去重，直接使用numpy生成的精确值）
        initial_poses = []
        for x_init in x_range:
            for y_init in y_range:
                for theta_init in angle_range:
                    initial_poses.append((float(x_init), float(y_init), float(theta_init)))
        
        total_poses = len(initial_poses)
        self.get_logger().info(f'   实际搜索: {total_poses} 个位姿')
        
        # 重置计数器
        with self.lock:
            self.searched_poses_count = 0
        
        # 使用线程池并行搜索
        max_workers = 8  # 使用8个线程
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            # 提交所有任务
            future_to_pose = {
                executor.submit(self.icp_match, scan_points, self.map_points, pose): pose 
                for pose in initial_poses
            }
            
            # 收集结果
            for future in as_completed(future_to_pose):
                result = future.result()
                
                # 实时更新计数
                with self.lock:
                    self.searched_poses_count += 1
                    if self.searched_poses_count % 10 == 0 or self.searched_poses_count == len(initial_poses):
                        self.get_logger().info(f'   进度: {self.searched_poses_count}/{len(initial_poses)}')
                
                if result[0] is not None:
                    pose, error, valid_count = result
                    search_results.append((pose, error, valid_count))
                    
                    # 优先选择误差小且匹配点多的结果
                    if error < best_error and valid_count > len(scan_points) * 0.3:
                        best_pose = pose
                        best_error = error
                        best_valid_count = valid_count
        
        # 显示前3个最佳结果
        if search_results:
            search_results.sort(key=lambda x: x[1])  # 按误差排序
            self.get_logger().info(f'   搜索完成,共 {len(search_results)} 个有效结果')
            for i, (pose, err, vc) in enumerate(search_results[:3]):
                self.get_logger().info(f'   候选{i+1}: 位置({pose[0]:.3f}, {pose[1]:.3f}), 角度{math.degrees(pose[2]):.1f}°, 误差{err:.4f}, 匹配点{vc}/{len(scan_points)}')
        
        # 判断是否有满足严格阈值的结果
        strict_threshold_met = (best_pose is not None and 
                                best_error < 0.01 and 
                                best_valid_count > len(scan_points) * 0.4)
        
        # 如果有结果（即使不满足严格阈值也发布最优的）
        if best_pose is not None:
            x, y, theta = best_pose
            
            if strict_threshold_met:
                self.get_logger().info('=' * 70)
                self.get_logger().info('✅ ICP 重定位成功！(高置信度)')
                self.get_logger().info(f'   位置: ({x:.3f}, {y:.3f}) m')
                self.get_logger().info(f'   角度: {math.degrees(theta):.1f}°')
                self.get_logger().info(f'   匹配误差: {best_error:.4f}')
                self.get_logger().info(f'   匹配点数: {best_valid_count}/{len(scan_points)} ({100*best_valid_count/len(scan_points):.1f}%)')
                self.get_logger().info('=' * 70)
            else:
                self.get_logger().info('=' * 70)
                self.get_logger().info('⚠️  选择最优匹配结果发布 (低置信度)')
                self.get_logger().info(f'   位置: ({x:.3f}, {y:.3f}) m')
                self.get_logger().info(f'   角度: {math.degrees(theta):.1f}°')
                self.get_logger().info(f'   匹配误差: {best_error:.4f} (理想<0.01)')
                self.get_logger().info(f'   匹配点数: {best_valid_count}/{len(scan_points)} ({100*best_valid_count/len(scan_points):.1f}%, 理想>40%)')
                self.get_logger().info('=' * 70)
            
            # 发布初始位姿（无论置信度高低都发布最优的）
            self.publish_initial_pose(x, y, theta)
            self.is_relocalized = True
            
            # 停止定时器
            self.relocalize_timer.cancel()
        else:
            self.get_logger().warn(f'⚠️  ICP 匹配完全失败,未找到任何有效结果')
    
    def publish_initial_pose(self, x, y, theta):
        """发布初始位姿"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # 四元数
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # 协方差（ICP结果高置信度）
        covariance = [0.0] * 36
        covariance[0] = 0.01    # x: ±10cm
        covariance[7] = 0.01    # y: ±10cm
        covariance[35] = 0.01   # yaw: ±5.7°
        msg.pose.covariance = covariance
        
        self.pose_pub.publish(msg)
        self.get_logger().info('📍 初始位姿已发布到 /initialpose')


def main(args=None):
    rclpy.init(args=args)
    node = ICPRelocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
