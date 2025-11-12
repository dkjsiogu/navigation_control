#!/usr/bin/env python3
"""
地图重发布节点 - 将 Transient Local 的地图转为 Volatile
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        
        # 订阅原始地图 (Transient Local)
        qos_transient = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_transient
        )
        
        # 重新发布地图 (Volatile) - 使用更稳定的 QoS
        qos_volatile = QoSProfile(
            depth=10,  # 增加队列深度
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map_viz',
            qos_volatile
        )
        
        self.map_received = False
        self.latest_map = None
        
        # 创建定时器，周期性重发布地图 (1Hz)
        self.timer = self.create_timer(1.0, self.republish_map)
        
        self.get_logger().info('地图重发布节点已启动: /map -> /map_viz')
    
    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info(f'首次收到地图: {msg.info.width}x{msg.info.height}')
            self.get_logger().info(f'地图原点: [{msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f}]')
            self.get_logger().info(f'地图分辨率: {msg.info.resolution:.3f} m/pixel')
            self.map_received = True
        
        self.latest_map = msg
        self.map_pub.publish(msg)
        self.get_logger().info('地图已更新', throttle_duration_sec=5.0)
    
    def republish_map(self):
        """定时重发布地图，确保订阅者始终能收到"""
        if self.latest_map is not None:
            # 更新时间戳
            self.latest_map.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.latest_map)

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
