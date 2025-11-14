#!/usr/bin/env python3
"""
任务控制器
==========
功能:
1. 管理机器人执行任务序列
2. 支持航点序列导航
3. 支持原地旋转、等待等操作
4. 监控导航状态

任务类型:
- goto: 前往航点
- rotate: 原地旋转到指定角度
- wait: 等待指定时间
- sequence: 执行航点序列

使用:
- 执行任务: ros2 service call /mission/execute std_srvs/srv/SetBool "{string: 'task_name'}"
- 停止任务: ros2 service call /mission/stop std_srvs/srv/Trigger
- 暂停/继续: ros2 service call /mission/pause std_srvs/srv/SetBool "{data: true}"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Int32
import yaml
import os
import math
import time
from enum import Enum

class TaskType(Enum):
    GOTO = 'goto'
    ROTATE = 'rotate'
    WAIT = 'wait'
    START_VISION = 'start_vision'
    STOP_VISION = 'stop_vision'
    SEQUENCE = 'sequence'

class MissionState(Enum):
    IDLE = 'idle'
    RUNNING = 'running'
    PAUSED = 'paused'
    COMPLETED = 'completed'
    FAILED = 'failed'

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # 参数
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.declare_parameter('missions_file', 'missions.yaml')
        self.declare_parameter('goal_reached_tolerance', 0.15)
        self.declare_parameter('rotation_tolerance', 0.1)  # 约6度
        
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.missions_file = self.get_parameter('missions_file').value
        self.goal_tolerance = self.get_parameter('goal_reached_tolerance').value
        self.rotation_tolerance = self.get_parameter('rotation_tolerance').value
        
        # 加载配置
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory('navigation_control')
        maps_dir = os.path.join(pkg_dir, 'maps')
        
        if not os.path.isabs(self.waypoints_file):
            self.waypoints_file = os.path.join(maps_dir, self.waypoints_file)
        if not os.path.isabs(self.missions_file):
            self.missions_file = os.path.join(maps_dir, self.missions_file)
        
        # 数据
        self.waypoints = {}
        self.missions = {}
        self.current_mission = None
        self.current_task_index = 0
        self.state = MissionState.IDLE
        self.current_pose = None
        
        # 加载航点和任务
        self.load_waypoints()
        self.load_missions()
        
        # 订阅
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # 订阅路径跟踪器状态
        self.goal_reached_sub = self.create_subscription(
            String, '/goal_status', self.goal_status_callback, 10)
        self.goal_reached = False
        
        # 发布
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.task_command_pub = self.create_publisher(Int32, '/task_command', 10)  # 发布视觉任务命令
        
        # 服务
        self.execute_srv = self.create_service(
            Trigger, '/mission/execute', self.execute_mission_callback)
        self.stop_srv = self.create_service(
            Trigger, '/mission/stop', self.stop_mission_callback)
        self.pause_srv = self.create_service(
            SetBool, '/mission/pause', self.pause_mission_callback)
        self.list_srv = self.create_service(
            Trigger, '/mission/list', self.list_missions_callback)
        
        # 订阅 - 用于传递任务名称
        self.mission_name_sub = self.create_subscription(
            String, '/mission/name', self.mission_name_callback, 10)
        
        self.pending_mission_name = None
        
        # 定时器 - 任务状态机
        self.timer = self.create_timer(0.1, self.mission_tick)
        
        # 任务执行相关
        self.task_start_time = None
        self.target_yaw = None
        self.last_goal_pose = None
        
        self.get_logger().info('🤖 任务控制器已启动')
        self.get_logger().info(f'   航点文件: {self.waypoints_file}')
        self.get_logger().info(f'   任务文件: {self.missions_file}')
        self.get_logger().info(f'   已加载任务: {len(self.missions)} 个')
        self.get_logger().info('')
        self.get_logger().info('📌 服务列表:')
        self.get_logger().info('   /mission/execute - 执行任务')
        self.get_logger().info('   /mission/stop    - 停止任务')
        self.get_logger().info('   /mission/pause   - 暂停/继续')
        self.get_logger().info('   /mission/list    - 列出任务')
        self.get_logger().info('')
        self.get_logger().info('💡 使用方法:')
        self.get_logger().info('   先发布名称: ros2 topic pub -1 /mission/name std_msgs/msg/String "{data: \'patrol_route\'}"')
        self.get_logger().info('   再调用服务: ros2 service call /mission/execute std_srvs/srv/Trigger')
    
    def mission_name_callback(self, msg):
        """接收任务名称"""
        self.pending_mission_name = msg.data
    
    def load_waypoints(self):
        """加载航点"""
        if not os.path.exists(self.waypoints_file):
            self.get_logger().warn('航点文件不存在')
            return
        
        try:
            with open(self.waypoints_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if data and 'waypoints' in data:
                    self.waypoints = data['waypoints']
                    self.get_logger().info(f'✅ 加载了 {len(self.waypoints)} 个航点')
        except Exception as e:
            self.get_logger().error(f'加载航点失败: {e}')
    
    def load_missions(self):
        """加载任务"""
        if not os.path.exists(self.missions_file):
            self.get_logger().info('任务文件不存在，创建示例任务')
            self.create_example_missions()
            return
        
        try:
            with open(self.missions_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if data and 'missions' in data:
                    self.missions = data['missions']
                    self.get_logger().info(f'✅ 加载了 {len(self.missions)} 个任务')
        except Exception as e:
            self.get_logger().error(f'加载任务失败: {e}')
    
    def create_example_missions(self):
        """创建示例任务文件"""
        example = {
            'missions': {
                'patrol_route': {
                    'description': '巡逻路线',
                    'tasks': [
                        {'type': 'goto', 'waypoint': 'point1'},
                        {'type': 'rotate', 'yaw': 3.14},  # 转180度
                        {'type': 'wait', 'duration': 2.0},
                        {'type': 'goto', 'waypoint': 'point2'},
                    ]
                }
            }
        }
        
        try:
            with open(self.missions_file, 'w', encoding='utf-8') as f:
                yaml.dump(example, f, allow_unicode=True, default_flow_style=False)
            self.get_logger().info(f'✅ 创建示例任务文件: {self.missions_file}')
        except Exception as e:
            self.get_logger().error(f'创建示例任务失败: {e}')
    
    def odom_callback(self, msg):
        """里程计回调"""
        self.current_pose = msg.pose.pose
    
    def goal_status_callback(self, msg):
        """路径跟踪器状态回调"""
        if msg.data == 'reached':
            self.goal_reached = True
            self.get_logger().info('📍 收到目标到达信号')
    
    def get_current_yaw(self):
        """获取当前航向角"""
        if self.current_pose is None:
            return None
        
        quat = self.current_pose.orientation
        yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
        return yaw
    
    def normalize_angle(self, angle):
        """归一化角度到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def execute_mission_callback(self, request, response):
        """执行任务"""
        mission_name = self.pending_mission_name
        self.pending_mission_name = None
        
        if not mission_name:
            response.success = False
            response.message = '❌ 请先指定任务名称 (/mission/name)'
            return response
        
        if mission_name not in self.missions:
            response.success = False
            response.message = f'❌ 任务 "{mission_name}" 不存在'
            return response
        
        if self.state == MissionState.RUNNING:
            response.success = False
            response.message = '❌ 已有任务正在执行'
            return response
        
        self.current_mission = self.missions[mission_name]
        self.current_task_index = 0
        self.state = MissionState.RUNNING
        
        response.success = True
        response.message = f'🚀 开始执行任务 "{mission_name}"'
        self.get_logger().info(response.message)
        
        return response
    
    def stop_mission_callback(self, request, response):
        """停止任务"""
        if self.state == MissionState.IDLE:
            response.success = True
            response.message = '⚠️  当前没有执行任务'
            return response
        
        self.state = MissionState.IDLE
        self.current_mission = None
        self.current_task_index = 0
        
        # 停止运动
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        response.success = True
        response.message = '⛔ 任务已停止'
        self.get_logger().info(response.message)
        
        return response
    
    def pause_mission_callback(self, request, response):
        """暂停/继续任务"""
        if request.data:  # 暂停
            if self.state == MissionState.RUNNING:
                self.state = MissionState.PAUSED
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                response.success = True
                response.message = '⏸️  任务已暂停'
            else:
                response.success = False
                response.message = '⚠️  没有正在执行的任务'
        else:  # 继续
            if self.state == MissionState.PAUSED:
                self.state = MissionState.RUNNING
                response.success = True
                response.message = '▶️  任务继续'
            else:
                response.success = False
                response.message = '⚠️  任务未暂停'
        
        self.get_logger().info(response.message)
        return response
    
    def list_missions_callback(self, request, response):
        """列出所有任务"""
        if not self.missions:
            response.success = True
            response.message = '📋 当前没有定义任务'
            return response
        
        msg = f'📋 已定义任务 ({len(self.missions)} 个):\n'
        for name, mission in self.missions.items():
            desc = mission.get('description', '无描述')
            task_count = len(mission.get('tasks', []))
            msg += f'  • {name}: {desc} ({task_count} 步)\n'
        
        response.success = True
        response.message = msg
        self.get_logger().info(msg)
        
        return response
    
    def mission_tick(self):
        """任务状态机"""
        if self.state != MissionState.RUNNING:
            return
        
        if self.current_mission is None:
            return
        
        tasks = self.current_mission.get('tasks', [])
        if self.current_task_index >= len(tasks):
            # 任务完成
            self.state = MissionState.COMPLETED
            self.get_logger().info('✅ 任务完成!')
            self.publish_status('completed')
            self.current_mission = None
            return
        
        # 执行当前任务
        task = tasks[self.current_task_index]
        task_type = task.get('type')
        
        # 调试：显示当前执行的任务
        if not hasattr(self, '_last_logged_task') or self._last_logged_task != self.current_task_index:
            self.get_logger().info(f'📌 执行任务 [{self.current_task_index + 1}/{len(tasks)}]: {task_type} - {task}')
            self._last_logged_task = self.current_task_index
        
        if task_type == 'goto':
            if self.execute_goto_task(task):
                self.get_logger().info(f'✅ 任务 {self.current_task_index + 1} 完成，进入下一步')
                self.current_task_index += 1
                self.task_start_time = None
                self._last_logged_task = None
        
        elif task_type == 'rotate':
            if self.execute_rotate_task(task):
                self.get_logger().info(f'✅ 任务 {self.current_task_index + 1} 完成，进入下一步')
                self.current_task_index += 1
                self.task_start_time = None
                self._last_logged_task = None
        
        elif task_type == 'wait':
            if self.execute_wait_task(task):
                self.get_logger().info(f'✅ 任务 {self.current_task_index + 1} 完成，进入下一步')
                self.current_task_index += 1
                self.task_start_time = None
                self._last_logged_task = None
        
        elif task_type == 'start_vision':
            if self.execute_start_vision_task(task):
                self.get_logger().info(f'✅ 任务 {self.current_task_index + 1} 完成，进入下一步')
                self.current_task_index += 1
                self._last_logged_task = None
        
        elif task_type == 'stop_vision':
            if self.execute_stop_vision_task(task):
                self.get_logger().info(f'✅ 任务 {self.current_task_index + 1} 完成，进入下一步')
                self.current_task_index += 1
                self._last_logged_task = None
    
    def execute_goto_task(self, task):
        """执行前往航点任务"""
        waypoint_name = task.get('waypoint')
        
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'❌ 航点 "{waypoint_name}" 不存在')
            self.state = MissionState.FAILED
            return True
        
        wp = self.waypoints[waypoint_name]
        
        # 第一次执行，发送目标
        if self.last_goal_pose != waypoint_name:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = wp['x']
            goal.pose.position.y = wp['y']
            goal.pose.position.z = 0.0
            
            yaw = wp['yaw']
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            
            self.goal_pub.publish(goal)
            self.last_goal_pose = waypoint_name
            self.goal_reached = False  # 重置目标到达标志
            self.get_logger().info(f'🎯 前往航点: {waypoint_name}')
            return False
        
        # 检查是否收到到达信号
        if self.goal_reached:
            self.get_logger().info(f'✅ 到达航点: {waypoint_name}')
            self.last_goal_pose = None
            self.goal_reached = False
            return True
        
        return False
    
    def execute_rotate_task(self, task):
        """执行原地旋转任务"""
        target_yaw = task.get('yaw', 0.0)
        
        if self.task_start_time is None:
            self.task_start_time = time.time()
            self.target_yaw = target_yaw
            self.get_logger().info(f'🔄 旋转到: {math.degrees(target_yaw):.1f}°')
        
        current_yaw = self.get_current_yaw()
        if current_yaw is None:
            return False
        
        # 计算角度差
        angle_diff = self.normalize_angle(target_yaw - current_yaw)
        
        if abs(angle_diff) < self.rotation_tolerance:
            # 到达目标角度
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info('✅ 旋转完成')
            return True
        
        # 发送旋转指令
        cmd = Twist()
        cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
        self.cmd_vel_pub.publish(cmd)
        
        return False
    
    def execute_wait_task(self, task):
        """执行等待任务"""
        duration = task.get('duration', 1.0)
        
        if self.task_start_time is None:
            self.task_start_time = time.time()
            self.get_logger().info(f'⏳ 等待 {duration} 秒')
        
        if time.time() - self.task_start_time >= duration:
            self.get_logger().info('✅ 等待完成')
            return True
        
        return False
    
    def execute_start_vision_task(self, task):
        """启动视觉任务"""
        task_id = task.get('task_id', 1)
        
        msg = Int32()
        msg.data = task_id
        self.task_command_pub.publish(msg)
        
        self.get_logger().info(f'🎥 启动视觉任务: {task_id}')
        return True
    
    def execute_stop_vision_task(self, task):
        """停止视觉任务"""
        msg = Int32()
        msg.data = 0
        self.task_command_pub.publish(msg)
        
        self.get_logger().info('🛑 停止视觉任务')
        return True
    
    def publish_status(self, status):
        """发布任务状态"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
