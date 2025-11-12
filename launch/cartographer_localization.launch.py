#!/usr/bin/env python3
"""
Cartographer 纯定位模式启动文件 (已集成轮式里程计)
功能:
1. RPLIDAR驱动 + 扫描过滤
2. 轮式里程计 (发布 /odom 和 TF: odom->base_link)
3. 加载已保存的 .pbstream 地图
4. Cartographer 纯定位模式（不建图，只定位）
5. 串口通信 (全向轮控制)
6. RViz2可视化

TF树: map -> odom (Cartographer) -> base_link (wheel_odom) -> laser (URDF)

Cartographer vs AMCL 的区别:
- Cartographer: 激光SLAM定位，精度高，自动全局定位
- AMCL: 粒子滤波定位，需要初始位姿，需要手动设置

使用方法:
1. 先用 mapping.launch.py 建图并保存 .pbstream
2. 运行此文件进行定位和导航:
   ros2 launch navigation_control cartographer_localization.launch.py
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
    pbstream_file_arg = DeclareLaunchArgument(
        'pbstream_file',
        default_value=os.path.join(maps_dir, 'my_map.pbstream'),
        description='Path to Cartographer .pbstream map file'
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
    
    return LaunchDescription([
        pbstream_file_arg,
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
                'enable_crc_check': False,  # 定位时关闭CRC检查以提高容错性
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
        
        # ============ Cartographer 纯定位节点 ============
        # 功能: 加载 .pbstream 地图，进行激光定位
        # 发布: map → odom TF (自动全局定位)
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
                '-configuration_basename', 'cartographer_localization.lua',
                '-load_state_filename', LaunchConfiguration('pbstream_file'),
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
        
        # ============ A* 路径规划器 (矩形footprint + 障碍物距离代价) ============
        Node(
            package='navigation_control',
            executable='astar_planner.py',
            name='astar_planner',
            output='screen',
            parameters=[{
                'robot_length': 0.262,         # 机器人长度 (262mm from URDF)
                'robot_width': 0.270,          # 机器人宽度 (270mm from URDF)
                'safety_margin': 0.03,         # 安全裕量 (3cm)
                'diagonal_penalty': 1.2,       # 对角线移动惩罚 (略微惩罚，允许圆弧)
                'smoothing_iterations': 20,    # 增加平滑迭代，生成更圆润的圆弧
                'waypoint_spacing': 0.20,      # 路径点间距 (20cm，保留转角点)
            }]
        ),
        
        # ============ 路径跟踪控制器 (Pure Pursuit + 全向轮) ============
        Node(
            package='navigation_control',
            executable='simple_goal_controller.py',
            name='path_tracker',
            output='screen',
            parameters=[{
                'max_linear_vel': 0.5,         # 最大线速度
                'max_angular_vel': 0.3,        # 最大角速度（仅用于保持朝向）
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
        ),
    ])
