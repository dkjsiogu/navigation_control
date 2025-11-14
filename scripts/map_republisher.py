#!/usr/bin/env python3
"""
地图重发布节点 - 智能融合老图和优化
老图作为基础（可以到之前的地方）
SLAM Toolbox 优化的部分覆盖到老图上
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        
        # QoS 配置
        qos_transient = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        qos_volatile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # 订阅老图 (map_server - 基础导航地图)
        self.static_map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.static_map_callback,
            qos_transient
        )
        
        # 订阅优化地图 (SLAM Toolbox - 实时优化)
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid,
            '/slam_map',
            self.slam_map_callback,
            qos_transient
        )
        
        # 发布融合后的地图
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map_viz',
            qos_volatile
        )
        
        self.static_map = None
        self.slam_map = None
        self.merged_map = None
        
        # 定时器，周期性发布融合地图 (5Hz)
        self.timer = self.create_timer(0.2, self.republish_map)
        
        self.get_logger().info('智能地图融合节点已启动:')
        self.get_logger().info('  基础: /map (老图, 可以到之前的地方)')
        self.get_logger().info('  优化: /slam_map (SLAM Toolbox 实时优化)')
        self.get_logger().info('  输出: /map_viz (融合后给 A*)')
        self.get_logger().info('  策略: 老图为基础, SLAM 优化覆盖')
    
    def static_map_callback(self, msg):
        if self.static_map is None:
            self.get_logger().info(f'📄 收到老图: {msg.info.width}x{msg.info.height}')
        self.static_map = msg
        self.merge_maps()
    
    def slam_map_callback(self, msg):
        if self.slam_map is None:
            self.get_logger().info(f'✨ 收到优化地图: {msg.info.width}x{msg.info.height}')
        self.slam_map = msg
        self.merge_maps()
    
    def merge_maps(self):
        """融合地图: 老图为基础, SLAM 优化的区域覆盖上去"""
        if self.static_map is None:
            return
        
        # 如果没有 SLAM 地图，直接用老图
        if self.slam_map is None:
            self.merged_map = self.static_map
            return
        
        # 检查地图是否匹配
        if (self.static_map.info.width != self.slam_map.info.width or
            self.static_map.info.height != self.slam_map.info.height):
            # 尺寸不匹配，只用老图
            self.merged_map = self.static_map
            return
        
        # 创建融合地图（复制老图）
        merged = OccupancyGrid()
        merged.header = self.static_map.header
        merged.info = self.static_map.info
        
        # 转换为 numpy 数组
        static_data = np.array(self.static_map.data, dtype=np.int8)
        slam_data = np.array(self.slam_map.data, dtype=np.int8)
        
        # 融合策略:
        # - 老图未知区域(-1): 保持未知
        # - 老图已知区域: 如果 SLAM 有更新(不是-1), 用 SLAM 的
        merged_data = static_data.copy()
        
        # SLAM 地图中已知的区域覆盖到老图上
        slam_known = slam_data != -1
        merged_data[slam_known] = slam_data[slam_known]
        
        merged.data = merged_data.tolist()
        self.merged_map = merged
    
    def republish_map(self):
        """定时重发布融合后的地图"""
        if self.merged_map is not None:
            # 更新时间戳并发布
            self.merged_map.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.merged_map)

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
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
