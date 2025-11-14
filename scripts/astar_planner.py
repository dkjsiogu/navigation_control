#!/usr/bin/env python3
"""
A* 路径规划器
功能:
1. 订阅 Cartographer 的栅格地图 (/map)
2. 订阅目标点 (/goal_pose)
3. 使用 A* 算法在栅格地图上规划路径
4. 发布规划好的路径 (/planned_path)
5. 路径平滑处理（减少折线，更适合全向轮）

作者: SLAM 大师们 🚀
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from heapq import heappush, heappop
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        # 参数 - 机器人实际尺寸（从URDF）
        self.declare_parameter('robot_length', 0.342)      # 机器人长度（米）前后
        self.declare_parameter('robot_width', 0.300)       # 机器人宽度（米）左右
        self.declare_parameter('safety_margin', 0.03)      # 安全裕量（米）
        self.declare_parameter('smoothing_iterations', 3)  # 路径平滑迭代次数（禁用平滑）
        self.declare_parameter('waypoint_spacing', 0.15)   # 路径点间距（米）
        self.declare_parameter('diagonal_penalty', 1.5)    # 斜向移动惩罚系数
        
        self.robot_length = self.get_parameter('robot_length').value
        self.robot_width = self.get_parameter('robot_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.smoothing_iterations = self.get_parameter('smoothing_iterations').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.diagonal_penalty = self.get_parameter('diagonal_penalty').value
        
        # 机器人半长和半宽（用于碰撞检测）
        self.robot_half_length = (self.robot_length + self.safety_margin) / 2.0
        self.robot_half_width = (self.robot_width + self.safety_margin) / 2.0
        
        # 对角线半径（用于快速碰撞初筛）
        self.robot_diagonal_radius = math.sqrt(self.robot_half_length**2 + self.robot_half_width**2)
        
        # 订阅地图 (使用map_viz而非map)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map_viz',
            self.map_callback,
            10
        )
        
        # 订阅目标点
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # 发布路径
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # 发布原始A*路径（未简化，用于调试）
        self.raw_path_pub = self.create_publisher(Path, '/raw_planned_path', 10)
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 地图数据
        self.map_data = None
        self.map_info = None
        
        self.get_logger().info('🚀 A* 路径规划器已启动 (矩形footprint模式)')
        self.get_logger().info(f'机器人尺寸: {self.robot_length*1000:.0f}mm × {self.robot_width*1000:.0f}mm')
        self.get_logger().info(f'安全裕量: {self.safety_margin*1000:.0f}mm')
        self.get_logger().info(f'斜向惩罚: {self.diagonal_penalty}x (优先直角移动)')
        self.get_logger().info(f'路径点间距: {self.waypoint_spacing}m')
    
    def map_callback(self, msg):
        """接收并处理地图"""
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        
        # 将地图数据转换为二维数组
        # OccupancyGrid: -1=未知, 0=自由, 100=占用
        self.map_data = np.array(msg.data).reshape((height, width))
        
        #self.get_logger().info(f'地图已更新: {width}x{height}, 分辨率={msg.info.resolution}m', 
        #                       throttle_duration_sec=5.0)
    
    def is_footprint_collision_free(self, x_world, y_world, yaw=0.0):
        """检查机器人矩形footprint是否与障碍物碰撞
        
        Args:
            x_world, y_world: 机器人中心世界坐标(米)
            yaw: 机器人航向角(弧度)，默认0（暂不考虑旋转）
        
        Returns:
            bool: 无碰撞返回True，有碰撞返回False
        """
        if self.map_info is None or self.map_data is None:
            return False
        
        # 计算机器人矩形的四个角点（世界坐标）
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        corners = [
            (self.robot_half_length, self.robot_half_width),   # 右前
            (self.robot_half_length, -self.robot_half_width),  # 右后
            (-self.robot_half_length, self.robot_half_width),  # 左前
            (-self.robot_half_length, -self.robot_half_width)  # 左后
        ]
        
        # 检查每个角点是否碰撞
        for local_x, local_y in corners:
            # 旋转到世界坐标系
            world_x = x_world + local_x * cos_yaw - local_y * sin_yaw
            world_y = y_world + local_x * sin_yaw + local_y * cos_yaw
            
            # 转换为栅格坐标
            grid_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
            grid_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)
            
            # 边界检查
            if grid_x < 0 or grid_x >= self.map_info.width or \
               grid_y < 0 or grid_y >= self.map_info.height:
                return False  # 超出地图范围
            
            # 碰撞检查（占用概率 > 50% 或未知区域）
            if self.map_data[grid_y, grid_x] > 50 or self.map_data[grid_y, grid_x] < 0:
                return False  # 碰撞或未知
        
        return True  # 所有角点都安全
    
    def goal_callback(self, msg):
        """接收目标点，规划路径"""
        if self.map_data is None:
            self.get_logger().warn('地图未就绪，无法规划路径')
            return
        
        # 获取当前位置
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            start_x = transform.transform.translation.x
            start_y = transform.transform.translation.y
        except TransformException as ex:
            self.get_logger().error(f'无法获取机器人位置: {ex}')
            return
        
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        self.get_logger().info(f'开始规划路径: ({start_x:.2f}, {start_y:.2f}) -> ({goal_x:.2f}, {goal_y:.2f})')
        
        # 世界坐标 -> 栅格坐标
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        if not self.is_valid_cell(start_grid) or not self.is_valid_cell(goal_grid):
            self.get_logger().error('起点或终点超出地图范围')
            return
        
        # 使用footprint检测起点终点
        start_world = self.grid_to_world(start_grid[0], start_grid[1])
        goal_world = self.grid_to_world(goal_grid[0], goal_grid[1])
        
        if not self.is_footprint_collision_free(start_world[0], start_world[1]):
            self.get_logger().warn('起点footprint与障碍物重叠，尝试规划...')
        
        if not self.is_footprint_collision_free(goal_world[0], goal_world[1]):
            self.get_logger().error('终点footprint与障碍物碰撞，无法规划')
            return
        
        # A* 路径规划
        path_grid = self.astar(start_grid, goal_grid)
        
        if path_grid is None:
            self.get_logger().error('路径规划失败，无法找到可行路径')
            return
        
        self.get_logger().info(f'路径规划成功，共 {len(path_grid)} 个栅格点')
        
        # 栅格路径 -> 世界坐标路径
        path_world = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
        
        # 发布原始A*路径（用于调试对比）
        self.publish_path(path_world, msg.header.frame_id, raw=True)
        
        # 路径插值（增加密度，让后续平滑生成圆弧）
        path_dense = self.interpolate_path(path_world)
        
        # 路径平滑（生成圆弧过渡）
        path_smooth = self.smooth_path(path_dense)
        
        # 智能抽稀（保留转角点，简化直线段）
        path_simplified = self.simplify_path_with_curvature(path_smooth)
        
        self.get_logger().info(f'路径处理完成: 原始{len(path_grid)}点 -> 插值{len(path_dense)}点 -> 平滑{len(path_smooth)}点 -> 简化{len(path_simplified)}点')
        
        # 发布最终路径
        self.publish_path(path_simplified, msg.header.frame_id, raw=False)
    
    def get_obstacle_distance_cost(self, grid_x, grid_y):
        """计算栅格点到最近障碍物的距离代价（距离越近代价越高，让路径往外绕）"""
        # 搜索半径：机器人对角线的2倍范围
        search_radius = int((self.robot_diagonal_radius * 2.0) / self.map_info.resolution)
        
        height, width = self.map_data.shape
        min_obstacle_dist = search_radius + 1  # 初始化为最大值
        
        # 在搜索半径内找最近障碍物
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                # 边界检查
                if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                    continue
                
                # 如果是障碍物
                if self.map_data[check_y, check_x] > 50:
                    dist = math.sqrt(dx*dx + dy*dy)
                    if dist < min_obstacle_dist:
                        min_obstacle_dist = dist
        
        # 距离转换为代价（反比关系）：距离越近，代价越高
        # 使用指数衰减：cost = scale * exp(-distance / decay_factor)
        if min_obstacle_dist <= search_radius:
            # 归一化距离 [0, 1]
            normalized_dist = min_obstacle_dist / search_radius
            # 代价：距离0时最高(5.0)，距离search_radius时接近0
            cost = 5.0 * math.exp(-3.0 * normalized_dist)  # 指数衰减
            return cost
        
        return 0.0  # 距离足够远，无额外代价
    
    def astar(self, start, goal):
        """A* 算法（使用footprint碰撞检测 + 障碍物距离代价）"""
        height, width = self.map_data.shape
        
        # 启发式函数：欧氏距离
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        # 8方向移动（优先直角移动，惩罚对角线）
        neighbors = [
            (1, 0, 1.0),                          # 右
            (-1, 0, 1.0),                         # 左
            (0, 1, 1.0),                          # 下
            (0, -1, 1.0),                         # 上
            (1, 1, 1.414 * self.diagonal_penalty),   # 右下 (惩罚)
            (-1, 1, 1.414 * self.diagonal_penalty),  # 左下 (惩罚)
            (1, -1, 1.414 * self.diagonal_penalty),  # 右上 (惩罚)
            (-1, -1, 1.414 * self.diagonal_penalty)  # 左上 (惩罚)
        ]
        
        # 优先队列: (f值, 计数器, 节点)
        open_set = []
        counter = 0
        heappush(open_set, (0, counter, start))
        
        # 记录路径
        came_from = {}
        
        # g值: 从起点到该点的实际代价
        g_score = {start: 0}
        
        # f值: g + h
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            current_f, _, current = heappop(open_set)
            
            # 到达目标
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # 探索邻居
            for dx, dy, cost in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 检查是否有效
                if not self.is_valid_cell(neighbor):
                    continue
                
                # 使用footprint碰撞检测
                neighbor_world = self.grid_to_world(neighbor[0], neighbor[1])
                if not self.is_footprint_collision_free(neighbor_world[0], neighbor_world[1]):
                    continue
                
                # 计算新的g值 + 障碍物距离代价（让路径往外绕）
                obstacle_cost = self.get_obstacle_distance_cost(neighbor[0], neighbor[1])
                tentative_g = g_score[current] + cost + obstacle_cost
                
                # 如果找到更好的路径
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    
                    counter += 1
                    heappush(open_set, (f_score[neighbor], counter, neighbor))
        
        # 没有找到路径
        return None
    
    def interpolate_path(self, path):
        """路径插值 - 在原始路径点之间插入中间点，增加密度"""
        if len(path) < 2:
            return path
        
        interpolated = [path[0]]
        
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            
            # 计算两点距离
            dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            
            # 插值间隔：机器人对角线的1/3（确保足够密集）
            interp_interval = self.robot_diagonal_radius / 3.0
            num_points = max(int(dist / interp_interval), 1)
            
            # 在两点之间插值
            for j in range(1, num_points):
                t = j / num_points
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                interpolated.append((x, y))
            
            # 添加p2（除了最后一个点）
            if i < len(path) - 1:
                interpolated.append(p2)
        
        # 添加终点
        interpolated.append(path[-1])
        
        return interpolated
    
    def smooth_path(self, path):
        """路径平滑（梯度下降法）- 带碰撞检测"""
        if len(path) < 3:
            return path
        
        smoothed = [list(p) for p in path]  # 转换为可修改的列表
        
        for _ in range(self.smoothing_iterations):
            for i in range(1, len(smoothed) - 1):
                # 保存原始位置
                original_x, original_y = smoothed[i][0], smoothed[i][1]
                
                # 平滑公式: p[i] = p[i] + α * (p[i-1] - p[i]) + β * (p[i+1] - p[i])
                alpha = 0.5  # 增加平滑强度，生成圆弧过渡
                beta = 0.5
                
                new_x = smoothed[i][0] + alpha * (smoothed[i-1][0] - smoothed[i][0])
                new_x += beta * (smoothed[i+1][0] - smoothed[i][0])
                
                new_y = smoothed[i][1] + alpha * (smoothed[i-1][1] - smoothed[i][1])
                new_y += beta * (smoothed[i+1][1] - smoothed[i][1])
                
                # 检查平滑后的点footprint是否碰撞
                if self.is_footprint_collision_free(new_x, new_y):
                    # 安全，可以平滑
                    smoothed[i][0] = new_x
                    smoothed[i][1] = new_y
                else:
                    # 不安全，保持原位置
                    smoothed[i][0] = original_x
                    smoothed[i][1] = original_y
        
        return [tuple(p) for p in smoothed]
    
    def simplify_path(self, path):
        """路径抽稀 - 带碰撞检测，确保直线段不穿过障碍物"""
        if len(path) < 3:
            return path
        
        simplified = [path[0]]  # 起点
        current_idx = 0
        
        while current_idx < len(path) - 1:
            # 从当前点开始，尝试找到最远的可直达点
            farthest_valid_idx = current_idx + 1
            
            for test_idx in range(len(path) - 1, current_idx, -1):
                # 检查从 current_idx 到 test_idx 的直线是否安全
                if self.is_line_collision_free(path[current_idx], path[test_idx]):
                    # 同时检查距离是否满足间距要求
                    dist = math.sqrt(
                        (path[test_idx][0] - path[current_idx][0])**2 + 
                        (path[test_idx][1] - path[current_idx][1])**2
                    )
                    if dist >= self.waypoint_spacing:
                        farthest_valid_idx = test_idx
                        break
            
            # 如果没找到满足间距的点，至少前进一步
            if farthest_valid_idx == current_idx + 1:
                # 检查相邻点是否可达
                if not self.is_line_collision_free(path[current_idx], path[current_idx + 1]):
                    # 相邻点都不可达（理论上不应该发生）
                    farthest_valid_idx = current_idx + 1
            
            simplified.append(path[farthest_valid_idx])
            current_idx = farthest_valid_idx
        
        # 确保终点在路径中
        if simplified[-1] != path[-1]:
            simplified.append(path[-1])
        
        return simplified
    
    def simplify_path_with_curvature(self, path):
        """基于曲率的智能路径简化 - 保留转角点，简化直线段"""
        if len(path) < 3:
            return path
        
        # 计算每个点的曲率（使用相邻三点）
        curvatures = [0.0]  # 起点曲率为0
        
        for i in range(1, len(path) - 1):
            p0 = path[i - 1]
            p1 = path[i]
            p2 = path[i + 1]
            
            # 向量
            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])
            
            # 计算转角（叉乘）
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            len1 = math.sqrt(v1[0]**2 + v1[1]**2) + 1e-6
            len2 = math.sqrt(v2[0]**2 + v2[1]**2) + 1e-6
            
            # 曲率近似为转角大小
            curvature = abs(cross) / (len1 * len2)
            curvatures.append(curvature)
        
        curvatures.append(0.0)  # 终点曲率为0
        
        # 标记需要保留的点
        keep = [True]  # 起点必须保留
        
        # 曲率阈值：超过此值认为是转角点
        curvature_threshold = 0.1
        
        for i in range(1, len(path) - 1):
            # 转角点必须保留
            if curvatures[i] > curvature_threshold:
                keep.append(True)
            else:
                # 直线段上的点，检查是否可以跳过
                # 找到上一个保留点
                last_kept = len(keep) - 1 - keep[::-1].index(True)
                
                # 如果距离上一个保留点太远，也要保留
                dist = math.sqrt(
                    (path[i][0] - path[last_kept][0])**2 + 
                    (path[i][1] - path[last_kept][1])**2
                )
                
                if dist >= self.waypoint_spacing:
                    keep.append(True)
                else:
                    keep.append(False)
        
        keep.append(True)  # 终点必须保留
        
        # 构建简化路径
        simplified = [path[i] for i in range(len(path)) if keep[i]]
        
        # 碰撞检测：确保简化后的直线段不穿过障碍物
        final_path = [simplified[0]]
        
        for i in range(1, len(simplified)):
            if self.is_line_collision_free(final_path[-1], simplified[i]):
                # 可以直达，但要检查是否跳过了重要转角
                # 查找两点之间是否有高曲率点
                start_idx = path.index(final_path[-1])
                end_idx = path.index(simplified[i])
                
                has_sharp_turn = False
                for j in range(start_idx + 1, end_idx):
                    if curvatures[j] > curvature_threshold * 1.5:  # 更严格的阈值
                        has_sharp_turn = True
                        break
                
                if has_sharp_turn:
                    # 有急转弯，不能跳过，添加中间的转角点
                    for j in range(start_idx + 1, end_idx + 1):
                        if curvatures[j] > curvature_threshold and path[j] not in final_path:
                            final_path.append(path[j])
                
                final_path.append(simplified[i])
            else:
                # 不能直达，添加中间点
                final_path.append(simplified[i])
        
        return final_path
    
    def is_line_collision_free(self, start_point, end_point):
        """检查两点之间的直线是否与障碍物碰撞（使用footprint检测）"""
        # 计算直线长度和采样间隔
        dist = math.sqrt((end_point[0] - start_point[0])**2 + 
                        (end_point[1] - start_point[1])**2)
        
        # 采样间隔：机器人对角线半径的一半（确保不遗漏碰撞）
        sample_interval = self.robot_diagonal_radius / 2.0
        num_samples = max(int(dist / sample_interval), 2)
        
        # 沿直线采样检查footprint碰撞
        for i in range(num_samples + 1):
            t = i / num_samples
            x = start_point[0] + t * (end_point[0] - start_point[0])
            y = start_point[1] + t * (end_point[1] - start_point[1])
            
            if not self.is_footprint_collision_free(x, y):
                return False
        
        return True
    
    def _bresenham_line_check_legacy(self, start_point, end_point):
        """废弃：Bresenham单点检测（保留供参考）"""
        # 世界坐标转栅格坐标
        start_grid = self.world_to_grid(start_point[0], start_point[1])
        end_grid = self.world_to_grid(end_point[0], end_point[1])
        
        # Bresenham直线算法
        x0, y0 = start_grid
        x1, y1 = end_grid
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # 检查当前点是否在障碍物内
            if not self.is_valid_cell((x, y)):
                return False
            if self.map_data[y, x] > 50:
                return False
            
            # 到达终点
            if x == x1 and y == y1:
                break
            
            # 移动到下一个点
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True
    
    def publish_path(self, path_world, frame_id='map', raw=False):
        """发布路径到 /planned_path 或 /raw_planned_path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        
        for x, y in path_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # 无旋转
            path_msg.poses.append(pose)
        
        if raw:
            self.raw_path_pub.publish(path_msg)
            self.get_logger().info(f'原始A*路径已发布到 /raw_planned_path: {len(path_msg.poses)} 点')
        else:
            self.path_pub.publish(path_msg)
            self.get_logger().info(f'最终路径已发布到 /planned_path: {len(path_msg.poses)} 点')
    

    
    def world_to_grid(self, x, y):
        """世界坐标 -> 栅格坐标"""
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x, grid_y):
        """栅格坐标 -> 世界坐标"""
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        
        world_x = origin_x + (grid_x + 0.5) * resolution
        world_y = origin_y + (grid_y + 0.5) * resolution
        
        return (world_x, world_y)
    
    def is_valid_cell(self, cell):
        """检查栅格坐标是否有效"""
        if self.map_info is None:
            return False
        
        x, y = cell
        return 0 <= x < self.map_info.width and 0 <= y < self.map_info.height

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
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
