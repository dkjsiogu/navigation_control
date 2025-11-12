#!/usr/bin/env python3
"""
建图模式启动文件 (已集成轮式里程计)
启动组件:
1. RPLIDAR驱动 + 扫描过滤器
2. Cartographer SLAM
3. 轮式里程计节点 (发布 /odom 和 TF: odom->base_link)
4. 串口通信 (下位机双向通信)
5. 键盘遥控 (WASD控制 0.5m/s)
6. Mapping Manager (地图保存)
7. RViz2可视化

使用方法:
    ros2 launch navigation_control mapping.launch.py
    
键盘控制:
    W/S - 前进/后退
    A/D - 左移/右移
    Q/E - 左转/右转
    Space - 急停
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
    
    dev_board_baudrate_arg = DeclareLaunchArgument(
        'dev_board_baudrate',
        default_value='115200',
        description='Development board baudrate'
    )
    
    use_keyboard_arg = DeclareLaunchArgument(
        'use_keyboard',
        default_value='true',
        description='Enable keyboard teleop (WASD control)'
    )
    
    return LaunchDescription([
        lidar_port_arg,
        dev_board_port_arg,
        dev_board_baudrate_arg,
        use_keyboard_arg,
        
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
        
        # ============ RPLIDAR驱动 ============
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
                ('/scan', '/scan_raw'),  # 重命名原始扫描
            ],
        ),
        
        # ============ 激光扫描过滤器 ============
        # 雷达倒装(X朝后Y朝右)，需过滤机器人后方本体
        Node(
            package='navigation_control',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'filter_angle_min': -2.30,  # -132° (左后角，雷达坐标系)
                'filter_angle_max': 2.69,   # 154° (右后角，雷达坐标系)
                'filter_range_max': 0.35,   # 只过滤 0.35m 以内的点
                'input_topic': '/scan_raw',
                'output_topic': '/scan',  # 输出到标准话题
            }],
        ),
        
        # ============ 轮式里程计节点 (发布 /odom 和 TF) ============
        Node(
            package='navigation_control',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'enable_crc_check': False,  # 建图时关闭CRC检查以提高容错性
            }],
        ),
        
        # ============ Cartographer SLAM ============
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_2d.lua'
            ],
            remappings=[
                ('/odom', '/odom'),  # 订阅轮式里程计数据
            ],
        ),
        
        # Cartographer占用栅格节点
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05'],
        ),
        
        # ============ 串口通信 (双向通信) ============
        Node(
            package='navigation_control',
            executable='serial_communication',
            name='serial_communication',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('dev_board_port'),
                'baudrate': LaunchConfiguration('dev_board_baudrate'),
                'timeout_ms': 100,
                'auto_reconnect': True,
                'reconnect_interval': 5.0,
            }],
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
        
        # ============ 键盘遥控 (WASD控制) ============
        Node(
            package='navigation_control',
            executable='keyboard_teleop.py',
            name='keyboard_teleop',
            output='screen',
            prefix='xterm -e',  # 在新终端窗口打开（需要安装xterm）
            condition=IfCondition(LaunchConfiguration('use_keyboard')),
        ),
        
        # ============ 地图保存管理器 ============
        Node(
            package='navigation_control',
            executable='mapping_manager',
            name='mapping_manager',
            output='screen',
            parameters=[{
                'map_save_directory': os.path.join(nav_control_dir, 'maps'),
                'map_name': 'my_map',
                'auto_save_interval': 60.0,
            }],
        ),
        
        # ============ RViz2可视化 ============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'mapping.rviz')],
        ),
    ])
