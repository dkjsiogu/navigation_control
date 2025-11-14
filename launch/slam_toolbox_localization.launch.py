#!/usr/bin/env python3
"""
SLAM Toolbox 纯定位模式启动文件
================================

特性:
1. 加载已保存的地图 (posegraph格式)
2. 纯定位模式 (不建新图)
3. 自动初始定位
4. 稳定的扫描匹配
5. 完整的导航功能

TF树: map -> odom (SLAM Toolbox) -> base_link (轮式里程计) -> laser (URDF)

使用方法:
1. 先用SLAM Toolbox建图并保存为 .posegraph
2. 将地图文件放在 maps/ 目录
3. 启动: ros2 launch navigation_control slam_toolbox_localization.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    nav_control_dir = get_package_share_directory('navigation_control')
    config_dir = os.path.join(nav_control_dir, 'config')
    maps_dir = os.path.join(nav_control_dir, 'maps')
    urdf_file = os.path.join(nav_control_dir, 'urdf', 'robot.urdf')
    slam_toolbox_config = os.path.join(config_dir, 'slam_toolbox_localization.yaml')
    
    # 读取 URDF 文件
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 声明启动参数
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(maps_dir, 'my_slam_map.yaml'),
        description='Path to map yaml file (pgm+yaml format)'
    )
    
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
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )
    
    use_icp_arg = DeclareLaunchArgument(
        'use_icp',
        default_value='true',
        description='Whether to use ICP auto-relocalization'
    )
    

    
    return LaunchDescription([
        map_yaml_arg,
        lidar_port_arg,
        dev_board_port_arg,
        use_rviz_arg,
        use_icp_arg,
        
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
                ('/scan', '/scan_raw'),
            ],
        ),
        
        # ============ 激光扫描过滤器 ============
        Node(
            package='navigation_control',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'filter_angle_min': -2.30,
                'filter_angle_max': 2.69,
                'filter_range_max': 0.35,
                'input_topic': '/scan_raw',
                'output_topic': '/scan',
            }],
        ),
        
        # ============ 地图服务器 (加载pgm+yaml地图) ============
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_yaml'),
                'use_sim_time': False,
            }],
        ),
        
        # ============ 生命周期管理器 ============
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        
        # ============ SLAM Toolbox 纯定位节点 (动态维护地图) ============
        # 发布到 /slam_map: 基于静态地图进行闭环优化和动态更新
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_toolbox_config],
            remappings=[
                ('/map', '/slam_map'),  # 动态优化的地图发布到 /slam_map
            ],
        ),
        
        # ============ ICP 自动重定位节点 (可选) ============
        Node(
            package='navigation_control',
            executable='icp_relocalization.py',
            name='icp_relocalization',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'map_topic': '/map_viz',
                'max_iterations': 50,
                'convergence_threshold': 0.001,
                'max_correspondence_distance': 0.5,
                'min_scan_points': 50,
                'initial_x': 0.0,
                'initial_y': 0.0,
                'search_grid_size': 0.2,            # ±0.2m 范围（从2m减少）
                'search_grid_resolution': 0.01,      # 1cm 精度（从1cm放大）
                'angle_search_range': 0.1745,       # ±10° (从±5°增加)
                'angle_search_step': 0.0872,        # 5° 步进（从1°放大）
                'auto_relocalize_interval': 3.0
            }],
            condition=IfCondition(LaunchConfiguration('use_icp'))
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
        
        # ============ A* 路径规划器 ============
        Node(
            package='navigation_control',
            executable='astar_planner.py',
            name='astar_planner',
            output='screen',
            parameters=[{
                'robot_length': 0.262,
                'robot_width': 0.270,
                'safety_margin': 0.03,
                'diagonal_penalty': 1.2,
                'smoothing_iterations': 20,
                'waypoint_spacing': 0.20,
            }]
        ),
        
        # ============ 路径跟踪控制器 ============
        Node(
            package='navigation_control',
            executable='simple_goal_controller.py',
            name='path_tracker',
            output='screen',
            parameters=[{
                'max_linear_vel': 0.5,
                'max_angular_vel': 0.3,
                'goal_tolerance': 0.10,
                'lookahead_distance': 0.5,
                'waypoint_tolerance': 0.15,
            }]
        ),
        
        # ============ 全向轮控制器 ============
        Node(
            package='navigation_control',
            executable='serial_data_publisher',
            name='serial_data_publisher',
            output='screen',
            parameters=[{
                'max_vx': 1.0,
                'max_vy': 1.0,
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
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
        
        # ============ 视觉任务调度器 ============
        Node(
            package='color_tracking_node',
            executable='task_scheduler',
            name='task_scheduler',
            output='screen',
        ),
        
        # ============ 颜色跟踪节点 (默认启动) ============
        Node(
            package='color_tracking_node',
            executable='color_tracking_node',
            name='color_tracking_node',
            output='screen',
        ),
    ])
