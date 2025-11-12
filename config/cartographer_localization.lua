-- Cartographer 纯定位配置文件
-- 使用已有的 .pbstream 地图进行定位，不进行建图

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser",        -- 跟踪激光雷达坐标系
  published_frame = "odom",         -- Cartographer 发布 map->odom
  odom_frame = "odom",
  provide_odom_frame = false,       -- 由轮式里程计发布 odom->base_link
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,  -- 启用里程计融合
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ========== 纯定位模式配置 ==========
-- 关键：禁用子地图插入，只做扫描匹配定位
MAP_BUILDER.num_background_threads = 4

-- 2D SLAM 配置
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- ========== 全局定位配置（重定位功能）==========
-- 启用在线相关扫描匹配（用于初始定位和重定位）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 实时相关扫描匹配器参数（初始定位后的跟踪）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- 增加搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- 增加角度搜索
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 激光扫描参数
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 2.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- ========== 纯定位关键配置 ==========
-- 禁用子地图构建（不添加新的子地图）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 999999  -- 设置非常大的值，实际不会建新图

-- 扫描匹配参数（用于定位）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0  -- 提高定位精度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- 运动滤波器（减少不必要的计算）
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- ========== 后端优化配置 ==========
-- 在纯定位模式下，后端优化主要用于平滑轨迹
POSE_GRAPH.optimize_every_n_nodes = 20  -- 更频繁优化
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5  -- 提高里程计权重
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

-- ========== 全局定位约束构建器（关键！）==========
-- 这是实现自动重定位的核心配置
POSE_GRAPH.constraint_builder.min_score = 0.55  -- 降低阈值以便初始匹配
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60  -- 全局定位最小分数

-- 全局采样参数（在整个地图中搜索匹配）
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 采样率（30%的子地图参与匹配）
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- 最大约束距离（米）
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- 快速相关扫描匹配器（用于全局搜索）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0  -- 大搜索窗口
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7  -- 搜索深度

-- 纯定位模式下禁用全局优化（可选）
-- POSE_GRAPH.optimize_every_n_nodes = 0  -- 如果不需要后端优化可以设为0

return options
