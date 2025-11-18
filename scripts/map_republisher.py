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
        
        # 订阅地图
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_transient
        )
        
        # 发布地图给 ICP 和 A*
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map_viz',
            qos_volatile
        )
        
        self.map_data = None
        
        # 定时器，周期性发布融合地图 (5Hz)
        self.timer = self.create_timer(0.2, self.republish_map)
        
        self.get_logger().info('地图重发布节点已启动')
        self.get_logger().info('  输入: /map')
        self.get_logger().info('  输出: /map_viz (供 ICP 和 A* 使用)')
    
    def map_callback(self, msg):
        if self.map_data is None:
            self.get_logger().info(f'📄 收到地图: {msg.info.width}x{msg.info.height}')
        self.map_data = msg
    
    def republish_map(self):
        """定时重发布地图"""
        if self.map_data is not None:
            self.map_data.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_data)

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
