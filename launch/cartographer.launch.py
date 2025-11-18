#!/usr/bin/env python3
"""
Cartographer SLAM 建图模式启动文件
功能:
1. RPLIDAR驱动 + 扫描过滤
2. 轮式里程计 (发布 /odom 和 TF: odom->base_link)
3. Cartographer 实时建图 + 定位
4. 串口通信 (全向轮控制)
5. 完整导航功能
6. RViz2可视化

TF树: map -> odom (Cartographer) -> base_link (wheel_odom) -> laser (URDF)

使用方法:
   ros2 launch navigation_control cartographer.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    nav_control_dir = get_package_share_directory('navigation_control')
    config_dir = os.path.join(nav_control_dir, 'config')
    maps_dir = os.path.join(nav_control_dir, 'maps')
    urdf_file = os.path.join(nav_control_dir, 'urdf', 'robot.urdf')
    
    # 读取 URDF 文件
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 声明启动参数
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/radar',
        description='LIDAR serial port'
    )
    
    dev_board_port_arg = DeclareLaunchArgument(
        'dev_board_port',
        default_value='/dev/stm32',
        description='Development board serial port'
    )
    
    return LaunchDescription([
        lidar_port_arg,
        dev_board_port_arg,
        
        # ============ 机器人模型发布 ============
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),
        
        # ============ 轮式里程计节点 (发布 /odom 和 TF: odom->base_link) ============
        Node(
            package='navigation_control',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'enable_crc_check': False,
                'enable_slam_correction': False,  # ❌ Cartographer模式禁用：Cartographer内部已做传感器融合
            }],
        ),
        
        # ============ 串口通信 (双向通信) ============
        Node(
            package='navigation_control',
            executable='serial_communication',
            name='serial_communication',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('dev_board_port'),
                'baudrate': 115200,
                'timeout_ms': 100,
                'auto_reconnect': True,
                'reconnect_interval': 5.0,
            }],
        ),
        
        # ============ 激光雷达 ============
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            remappings=[
                ('/scan', '/scan_raw'),  # 发布到 /scan_raw
            ],
        ),
        
        # ============ 激光扫描过滤器 (过滤机器人本体) ============
        # 雷达倒装(X朝后Y朝右)，需过滤机器人后方本体
        Node(
            package='navigation_control',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'filter_angle_min': -2.30,  # -132° (左后角，雷达坐标系)
                'filter_angle_max': 2.69,   # 154° (右后角，雷达坐标系)
                'filter_range_max': 0.35,   # 只过滤 0.35m 以内
                'input_topic': '/scan_raw',
                'output_topic': '/scan',    # 输出到标准 /scan
            }],
        ),
        
        # ============ Cartographer SLAM 建图节点 ============
        # 功能: 实时建图 + 定位
        # 发布: map → odom TF
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer.lua',
            ],
            remappings=[
                ('/odom', '/odom'),  # 订阅轮式里程计数据（融合定位）
            ],
        ),
        
        # ============ Cartographer 占用栅格节点 (发布地图) ============
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05,
            }],
        ),
        
        # ============ 地图重发布节点 (解决 RViz2 QoS 问题) ============
        Node(
            package='navigation_control',
            executable='map_republisher.py',
            name='map_republisher',
            output='screen',
        ),
        
        # ============ 航点管理器 ============
        Node(
            package='navigation_control',
            executable='waypoint_manager.py',
            name='waypoint_manager',
            output='screen',
            parameters=[{
                'waypoints_file': 'waypoints.yaml',
            }],
        ),
        
        # ============ 任务控制器 ============
        Node(
            package='navigation_control',
            executable='mission_controller.py',
            name='mission_controller',
            output='screen',
            parameters=[{
                'waypoints_file': 'waypoints.yaml',
                'missions_file': 'missions.yaml',
                'goal_reached_tolerance': 0.15,
                'rotation_tolerance': 0.1,
            }],
        ),
        
        # ============ A* 路径规划器 (矩形footprint + 障碍物距离代价) ============
        Node(
            package='navigation_control',
            executable='astar_planner.py',
            name='astar_planner',
            output='screen',
            parameters=[{
                'robot_length': 0.382,         # 机器人长度 (382mm URDF collision)
                'robot_width': 0.340,          # 机器人宽度 (340mm URDF collision)
                'safety_margin': 0.03,         # 安全裕量 (3cm)
                'diagonal_penalty': 1.2,       # 对角线移动惩罚 (略微惩罚，允许圆弧)
                'smoothing_iterations': 20,    # 增加平滑迭代，生成更圆润的圆弧
                'waypoint_spacing': 0.20,      # 路径点间距 (20cm，保留转角点)
            }]
        ),
        
        # ============ 路径跟踪控制器 (Pure Pursuit + 全向轮 - 8Hz雷达优化) ============
        Node(
            package='navigation_control',
            executable='simple_goal_controller.py',
            name='path_tracker',
            output='screen',
            parameters=[{
                'max_linear_vel': 0.3,         # 8Hz雷达最优巡航速度
                'max_angular_vel': 0.5,        # ~30°/s，避免IMU积分误差
                'goal_tolerance': 0.10,        # 到达目标容差 (10cm)
                'lookahead_distance': 0.5,     # Pure Pursuit 前瞻距离
                'waypoint_tolerance': 0.15,    # 路径点切换容差
            }]
        ),
        
        # ============ 全向轮控制器 (cmd_vel -> 下位机协议) ============
        Node(
            package='navigation_control',
            executable='serial_data_publisher',
            name='serial_data_publisher',
            output='screen',
            parameters=[{
                'max_vx': 0.3,
                'max_vy': 0.3,
                'max_wz': 2.0,
                'velocity_timeout': 1.0,
                'smooth_factor': 0.7,
            }],
        ),
        
        # ============ RViz2 可视化 ============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'navigation_debug.rviz')],
        ),
    ])

