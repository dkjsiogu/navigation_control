#!/usr/bin/env python3
"""
航点管理节点
============
功能:
1. 保存当前位置为航点 (服务)
2. 加载/保存航点到 YAML
3. 发布航点列表 (标记显示)
4. 提供航点查询服务

使用:
- 保存航点: ros2 service call /waypoint/save navigation_control_msgs/srv/SaveWaypoint "{name: 'point1'}"
- 列出航点: ros2 service call /waypoint/list std_srvs/srv/Trigger
- 删除航点: ros2 service call /waypoint/delete std_srvs/srv/SetBool "{data: true, string: 'point1'}"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
import yaml
import os
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # 参数
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.waypoints_file = self.get_parameter('waypoints_file').value
        
        # 完整路径
        if not os.path.isabs(self.waypoints_file):
            # 如果是相对路径，保存到工作空间 maps 目录
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('navigation_control')
            maps_dir = os.path.join(pkg_dir, 'maps')
            os.makedirs(maps_dir, exist_ok=True)
            self.waypoints_file = os.path.join(maps_dir, self.waypoints_file)
        
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 航点数据 {name: {'x': float, 'y': float, 'yaw': float, 'description': str}}
        self.waypoints = {}
        
        # 加载已有航点
        self.load_waypoints()
        
        # 服务
        self.save_srv = self.create_service(
            Trigger, '/waypoint/save', self.save_waypoint_callback)
        self.list_srv = self.create_service(
            Trigger, '/waypoint/list', self.list_waypoints_callback)
        self.delete_srv = self.create_service(
            Trigger, '/waypoint/delete', self.delete_waypoint_callback)
        self.goto_srv = self.create_service(
            Trigger, '/waypoint/goto', self.goto_waypoint_callback)
        self.apply_offset_srv = self.create_service(
            Trigger, '/waypoint/apply_relocalization_offset', self.apply_offset_callback)
        
        # 订阅 - 用于传递参数
        self.waypoint_name_sub = self.create_subscription(
            String, '/waypoint/name', self.waypoint_name_callback, 10)
        
        # 订阅重定位消息
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        
        self.pending_waypoint_name = None
        self.relocalization_pose = None  # 记录ICP给出的重定位位姿
        self.pre_relocalization_pose = None  # 记录重定位前的位姿
        
        # 发布器
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 定时发布航点标记
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('🗺️  航点管理器已启动')
        self.get_logger().info(f'   航点文件: {self.waypoints_file}')
        self.get_logger().info(f'   已加载航点: {len(self.waypoints)} 个')
        self.get_logger().info('')
        self.get_logger().info('📌 服务列表:')
        self.get_logger().info('   /waypoint/save   - 保存当前位置')
        self.get_logger().info('   /waypoint/list   - 列出所有航点')
        self.get_logger().info('   /waypoint/delete - 删除航点')
        self.get_logger().info('   /waypoint/goto   - 前往航点')
        self.get_logger().info('   /waypoint/apply_relocalization_offset - 应用ICP重定位偏移到所有航点')
        self.get_logger().info('')
        self.get_logger().info('💡 使用方法:')
        self.get_logger().info('   先发布名称: ros2 topic pub -1 /waypoint/name std_msgs/msg/String "{data: \'point1\'}"')
        self.get_logger().info('   再调用服务: ros2 service call /waypoint/save std_srvs/srv/Trigger')
    
    def waypoint_name_callback(self, msg):
        """接收航点名称"""
        self.pending_waypoint_name = msg.data
    
    def initialpose_callback(self, msg):
        """接收ICP重定位消息"""
        # 先记录当前的旧位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.5))
            
            self.pre_relocalization_pose = {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'yaw': math.atan2(
                    2.0 * (transform.transform.rotation.w * transform.transform.rotation.z + 
                           transform.transform.rotation.x * transform.transform.rotation.y),
                    1.0 - 2.0 * (transform.transform.rotation.y * transform.transform.rotation.y + 
                                 transform.transform.rotation.z * transform.transform.rotation.z)
                )
            }
        except Exception as e:
            self.get_logger().warn(f'无法获取重定位前位姿: {e}')
            return
        
        # 记录ICP给出的新位姿
        quat = msg.pose.pose.orientation
        self.relocalization_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
        }
        
        # 计算偏移量
        dx = self.relocalization_pose['x'] - self.pre_relocalization_pose['x']
        dy = self.relocalization_pose['y'] - self.pre_relocalization_pose['y']
        dyaw = self.relocalization_pose['yaw'] - self.pre_relocalization_pose['yaw']
        
        # 归一化角度
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 检测到ICP重定位!')
        self.get_logger().info(f'   旧位姿: ({self.pre_relocalization_pose["x"]:.3f}, {self.pre_relocalization_pose["y"]:.3f}, {math.degrees(self.pre_relocalization_pose["yaw"]):.1f}°)')
        self.get_logger().info(f'   新位姿: ({self.relocalization_pose["x"]:.3f}, {self.relocalization_pose["y"]:.3f}, {math.degrees(self.relocalization_pose["yaw"]):.1f}°)')
        self.get_logger().info(f'   偏移量: dx={dx:.3f}m, dy={dy:.3f}m, dyaw={math.degrees(dyaw):.1f}°')
        self.get_logger().info('   💡 执行命令更新航点: ros2 service call /waypoint/apply_relocalization_offset std_srvs/srv/Trigger')
        self.get_logger().info('=' * 60)
    
    def apply_offset_callback(self, request, response):
        """应用重定位偏移到所有航点"""
        if self.relocalization_pose is None or self.pre_relocalization_pose is None:
            response.success = False
            response.message = '❌ 没有检测到重定位偏移'
            return response
        
        if not self.waypoints:
            response.success = False
            response.message = '❌ 没有航点需要更新'
            return response
        
        # 计算偏移量
        dx = self.relocalization_pose['x'] - self.pre_relocalization_pose['x']
        dy = self.relocalization_pose['y'] - self.pre_relocalization_pose['y']
        dyaw = self.relocalization_pose['yaw'] - self.pre_relocalization_pose['yaw']
        
        # 归一化角度
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        # 应用偏移到所有航点
        updated_count = 0
        self.get_logger().info('更新航点:')
        for name, wp in self.waypoints.items():
            old_x, old_y, old_yaw = wp['x'], wp['y'], wp['yaw']
            
            # 应用平移
            wp['x'] = old_x + dx
            wp['y'] = old_y + dy
            
            # 应用旋转
            wp['yaw'] = old_yaw + dyaw
            
            # 归一化角度
            while wp['yaw'] > math.pi:
                wp['yaw'] -= 2 * math.pi
            while wp['yaw'] < -math.pi:
                wp['yaw'] += 2 * math.pi
            
            updated_count += 1
            self.get_logger().info(
                f'   {name}: ({old_x:.2f}, {old_y:.2f}, {math.degrees(old_yaw):.0f}°) → ({wp["x"]:.2f}, {wp["y"]:.2f}, {math.degrees(wp["yaw"]):.0f}°)')
        
        # 保存更新后的航点
        if self.save_waypoints_to_file():
            response.success = True
            response.message = f'✅ 已更新 {updated_count} 个航点 (偏移: dx={dx:.3f}m, dy={dy:.3f}m, dyaw={math.degrees(dyaw):.1f}°)'
            self.get_logger().info(response.message)
            
            # 清除记录
            self.relocalization_pose = None
            self.pre_relocalization_pose = None
        else:
            response.success = False
            response.message = '❌ 更新成功但保存文件失败'
        
        return response
    
    def get_current_pose(self):
        """获取机器人当前位姿 (map 坐标系)"""
        try:
            # 查询 map -> base_link 的变换
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # 四元数转欧拉角
            quat = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            return x, y, yaw
        
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'获取位姿失败: {e}')
            return None, None, None
    
    def load_waypoints(self):
        """从 YAML 加载航点"""
        if not os.path.exists(self.waypoints_file):
            self.get_logger().info(f'航点文件不存在，将创建: {self.waypoints_file}')
            return
        
        try:
            with open(self.waypoints_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if data and 'waypoints' in data:
                    self.waypoints = data['waypoints']
                    self.get_logger().info(f'✅ 加载了 {len(self.waypoints)} 个航点')
        except Exception as e:
            self.get_logger().error(f'加载航点失败: {e}')
    
    def save_waypoints_to_file(self):
        """保存航点到 YAML"""
        try:
            data = {'waypoints': self.waypoints}
            with open(self.waypoints_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
            return True
        except Exception as e:
            self.get_logger().error(f'保存航点失败: {e}')
            return False
    
    def save_waypoint_callback(self, request, response):
        """保存当前位置为航点"""
        waypoint_name = self.pending_waypoint_name if self.pending_waypoint_name else f'waypoint_{len(self.waypoints)+1}'
        self.pending_waypoint_name = None  # 清除
        
        x, y, yaw = self.get_current_pose()
        
        if x is None:
            response.success = False
            response.message = '❌ 无法获取当前位姿'
            return response
        
        # 保存航点
        self.waypoints[waypoint_name] = {
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'description': ''
        }
        
        # 写入文件
        if self.save_waypoints_to_file():
            response.success = True
            response.message = f'✅ 航点 "{waypoint_name}" 已保存: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = '❌ 保存到文件失败'
        
        return response
    
    def list_waypoints_callback(self, request, response):
        """列出所有航点"""
        if not self.waypoints:
            response.success = True
            response.message = '📍 当前没有保存的航点'
            return response
        
        msg = f'📍 已保存航点 ({len(self.waypoints)} 个):\n'
        for name, wp in self.waypoints.items():
            msg += f'  • {name}: ({wp["x"]:.2f}, {wp["y"]:.2f}, {math.degrees(wp["yaw"]):.1f}°)\n'
        
        response.success = True
        response.message = msg
        self.get_logger().info(msg)
        
        return response
    
    def delete_waypoint_callback(self, request, response):
        """删除航点"""
        waypoint_name = self.pending_waypoint_name
        self.pending_waypoint_name = None
        
        if not waypoint_name:
            response.success = False
            response.message = '❌ 请先指定航点名称 (/waypoint/name)'
            return response
        
        if waypoint_name in self.waypoints:
            del self.waypoints[waypoint_name]
            if self.save_waypoints_to_file():
                response.success = True
                response.message = f'✅ 航点 "{waypoint_name}" 已删除'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = '❌ 删除失败（文件写入错误）'
        else:
            response.success = False
            response.message = f'❌ 航点 "{waypoint_name}" 不存在'
        
        return response
    
    def goto_waypoint_callback(self, request, response):
        """前往指定航点"""
        waypoint_name = self.pending_waypoint_name
        self.pending_waypoint_name = None
        
        if not waypoint_name:
            response.success = False
            response.message = '❌ 请先指定航点名称 (/waypoint/name)'
            return response
        
        if waypoint_name not in self.waypoints:
            response.success = False
            response.message = f'❌ 航点 "{waypoint_name}" 不存在'
            return response
        
        wp = self.waypoints[waypoint_name]
        
        # 发布目标位姿
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.position.z = 0.0
        
        # 欧拉角转四元数
        yaw = wp['yaw']
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        
        self.goal_pub.publish(goal)
        
        response.success = True
        response.message = f'🚀 正在前往航点 "{waypoint_name}"'
        self.get_logger().info(response.message)
        
        return response
    
    def publish_markers(self):
        """发布航点可视化标记"""
        marker_array = MarkerArray()
        
        for i, (name, wp) in enumerate(self.waypoints.items()):
            # 球形标记
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
            
            # 文本标记
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = wp['x']
            text_marker.pose.position.y = wp['y']
            text_marker.pose.position.z = 0.3
            
            text_marker.scale.z = 0.15
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = name
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
