/**
 * @file wheel_odometry_node.cpp
 * @brief 轮式里程计节点 - 接收下位机速度数据并积分到世界坐标系
 * 
 * 功能:
 * 1. 订阅串口接收的底盘实时速度数据 (chassis_vx/vy/w)
 * 2. 使用ROS时间戳计算dt，对速度进行积分
 * 3. 将机器人坐标系的速度转换到世界坐标系并累加位姿
 * 4. 发布 nav_msgs/Odometry 消息到 /odom 话题
 * 5. 发布 TF 变换: odom -> base_link
 * 
 * 下位机数据包格式（与 master_process.h 中 Vision_Send_s 对应）:
 * - header: 0x5A (1字节)
 * - detect_color + flags: (1字节, 位域)
 * - roll: float (4字节) - IMU横滚角
 * - pitch: float (4字节) - IMU俯仰角
 * - yaw: float (4字节) - IMU航向角
 * - delta_theta: float (4字节) - 角度增量 (rad) [遗留字段，不再使用]
 * - disp_x: float (4字节) - 位移增量X [遗留字段，不再使用]
 * - disp_y: float (4字节) - 位移增量Y [遗留字段，不再使用]
 * - heading_diff: float (4字节) - 运动方向角 [遗留字段]
 * - chassis_vx: float (4字节) - ⚠️ 底盘实时速度X (m/s, 机器人坐标系前进方向)
 * - chassis_vy: float (4字节) - ⚠️ 底盘实时速度Y (m/s, 机器人坐标系左侧方向)
 * - chassis_w: float (4字节) - ⚠️ 底盘角速度 (rad/s, 逆时针为正)
 * - game_time: uint16_t (2字节) - 比赛时间 (s)
 * - timestamp: uint32_t (4字节) - 板载时间戳 (ms) [不使用，用ROS时间]
 * - checksum: uint16_t (2字节) - CRC16校验
 * 总计: 50字节 (更新于2025-01-08)
 * 
 * ⚠️ 架构变更 (2025-01-08):
 *    下位机现在发送实时速度而非50ms积分，上位机负责用ROS时间戳做积分
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstring>
#include <cmath>

using namespace std::chrono_literals;

// CRC16校验表 (与下位机一致)
static const uint16_t CRC16_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

#define CRC16_INIT 0xFFFF

// CRC16校验函数
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t ch_data;
    if (pchMessage == nullptr) return 0xFFFF;
    while (dwLength--) {
        ch_data = *pchMessage++;
        wCRC = ((uint16_t)(wCRC) >> 8) ^ CRC16_TABLE[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
    }
    return wCRC;
}

// 里程计数据包结构 - 与下位机 Vision_Send_s 完全对应
// 定义见 master_process.h (2025-01-08 更新: 新增底盘速度字段)
struct __attribute__((packed)) VisionSendPacket {
    uint8_t header;              // 0x5A
    uint8_t detect_color : 1;    // 0-red 1-blue
    uint8_t task_mode : 2;       // 0-auto 1-aim 2-buff
    uint8_t reset_tracker : 1;   // bool
    uint8_t is_play : 1;
    uint8_t change_target : 1;   // bool
    uint8_t reserved_bits : 2;
    float roll;                  // IMU姿态 (4字节)
    float pitch;                 // IMU姿态 (4字节)
    float yaw;                   // IMU姿态 (4字节)
    float delta_theta;           // 角度增量 (rad, 4字节)
    float disp_x;                // 位移增量X (m, 4字节)
    float disp_y;                // 位移增量Y (m, 4字节)
    float heading_diff;          // 运动方向角 (rad, 4字节)
    float chassis_vx;            // ⚠️ 新增: 底盘实时速度X (m/s, 4字节)
    float chassis_vy;            // ⚠️ 新增: 底盘实时速度Y (m/s, 4字节)
    float chassis_w;             // ⚠️ 新增: 底盘角速度 (rad/s, 4字节)
    double x;
    double y;
    uint16_t game_time;          // 比赛时间 (s, 2字节)
    uint32_t timestamp;          // 时间戳 (ms, 4字节)
    uint16_t checksum;           // CRC16校验 (2字节)
    // 总计: 1+1+4*10+2+4+2 = 50字节
    
    // 验证CRC16
    bool verifyCRC() const {
        uint16_t calculated_crc = Get_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t*>(this), 
            sizeof(VisionSendPacket) - 2,  // CRC计算除最后2字节外的所有字节
            CRC16_INIT
        );
        return calculated_crc == checksum;
    }
};

class WheelOdometryNode : public rclcpp::Node
{
public:
    WheelOdometryNode() : Node("wheel_odometry_node")
    {
        // 声明参数
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("enable_crc_check", true);
        this->declare_parameter("imu_drift_compensation_deg_per_min", 0.5);  // 每分钟补偿角度(度)
        this->declare_parameter("enable_slam_correction", true);  // 是否启用SLAM校正
        this->declare_parameter("slam_correction_interval", 3.0);  // SLAM校正间隔(秒) - 🔧 低频率避免噪声
        this->declare_parameter("slam_correction_static_threshold", 0.02);  // 静止判定阈值(m/s) - 2cm/s
        this->declare_parameter("slam_correction_moving_gain", 0.0);   // 运动时不校正
        this->declare_parameter("slam_correction_static_gain", 0.0);   // 静止时也不校正(默认) - 完全信任里程计
        this->declare_parameter("slam_correction_large_error_threshold", 0.30);  // 大误差阈值(m) - 30cm
        this->declare_parameter("slam_correction_large_error_gain", 0.20);  // 大误差校正增益(20%) - 一次性修正
        
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        enable_crc_check_ = this->get_parameter("enable_crc_check").as_bool();
        imu_drift_compensation_rate_ = this->get_parameter("imu_drift_compensation_deg_per_min").as_double() 
                                       * (M_PI / 180.0) / 60.0;  // 转换为 rad/s
        enable_slam_correction_ = this->get_parameter("enable_slam_correction").as_bool();
        slam_correction_interval_ = this->get_parameter("slam_correction_interval").as_double();
        slam_static_threshold_ = this->get_parameter("slam_correction_static_threshold").as_double();
        slam_moving_gain_ = this->get_parameter("slam_correction_moving_gain").as_double();
        slam_static_gain_ = this->get_parameter("slam_correction_static_gain").as_double();
        slam_large_error_threshold_ = this->get_parameter("slam_correction_large_error_threshold").as_double();
        slam_large_error_gain_ = this->get_parameter("slam_correction_large_error_gain").as_double();
        
        // 订阅串口接收数据
        serial_rx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "serial_rx_data", 10,
            std::bind(&WheelOdometryNode::serialRxCallback, this, std::placeholders::_1));
        
        // 发布里程计
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
        
        // 发布解析后的里程计数据（用于调试）
        odom_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "wheel_odom_data", 10);
        
        // TF广播器
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
        
        // TF监听器 - 用于获取SLAM校正
        if (enable_slam_correction_) {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            
            // 定时器 - 定期从SLAM获取校正
            slam_correction_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(slam_correction_interval_),
                std::bind(&WheelOdometryNode::correctFromSlam, this));
        }
        
        // 服务 - 重置里程计
        reset_odom_srv_ = this->create_service<std_srvs::srv::Empty>(
            "reset_odometry",
            std::bind(&WheelOdometryNode::resetOdometryCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 定时器 - 发布统计信息
        //stats_timer_ = this->create_wall_timer(
        //    5s, std::bind(&WheelOdometryNode::publishStats, this));
        
        // 初始化位姿
        resetOdometry();
        
        // 记录启动时间（用于IMU漂移补偿计算）
        imu_compensation_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "===================================");
        RCLCPP_INFO(this->get_logger(), "轮式里程计节点已启动");
        RCLCPP_INFO(this->get_logger(), "Odom frame: %s", odom_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish TF: %s", publish_tf_ ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "CRC Check: %s", enable_crc_check_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "SLAM Correction: %s (%.2fs interval)", 
                    enable_slam_correction_ ? "ENABLED" : "DISABLED", slam_correction_interval_);
        if (enable_slam_correction_) {
            RCLCPP_INFO(this->get_logger(), "  └─ 静止阈值: %.2fm/s | 运动增益: %.1f%% | 静止增益: %.1f%%",
                       slam_static_threshold_, slam_moving_gain_*100, slam_static_gain_*100);
        }
        RCLCPP_INFO(this->get_logger(), "IMU Drift Compensation: %.2f°/min (%.6f rad/s)", 
                   imu_drift_compensation_rate_ * 60.0 * 180.0 / M_PI, imu_drift_compensation_rate_);
        RCLCPP_INFO(this->get_logger(), "===================================");
    }

private:
    // 串口接收回调
    void serialRxCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        // 检查数据长度
        if (msg->data.size() != sizeof(VisionSendPacket)) {
            return;  // 不是里程计数据包，忽略
        }
        
        // 解析数据包
        VisionSendPacket packet;
        std::memcpy(&packet, msg->data.data(), sizeof(packet));
        
                
        // 验证帧头
        if (packet.header != 0x5A) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Invalid header: 0x%02X (expected 0x5A)",
                               packet.header);
            packets_invalid_++;
            return;
        }
        
        // 验证CRC16 (可选)
        if (enable_crc_check_ && !packet.verifyCRC()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "CRC16校验失败");
            packets_crc_error_++;
            return;
        }
        
        // 处理里程计数据
        processOdometryDelta(packet);
        
        packets_received_++;
    }
    
    // 处理里程计增量数据
    void processOdometryDelta(const VisionSendPacket& packet)
    {
        // ===== 新架构：下位机1ms定时器累加位移，上位机计算差值 =====
        // packet.x/y: 下位机累积的位移 (m, 机器人坐标系)
        // packet.chassis_vx/vy: 实时速度 (m/s, 用于验证对比)
        
        // 1. 计算时间增量 (使用ROS时间)
        rclcpp::Time current_time = this->now();
        
        if (!last_update_time_.nanoseconds()) {
            // 第一帧：只初始化时间和下位机位移基准，不积分
            last_update_time_ = current_time;
            last_board_x_ = packet.x;
            last_board_y_ = packet.y;
            last_board_pos_valid_ = true;
            RCLCPP_INFO(this->get_logger(), "里程计初始化：下位机位移基准 (%.6f, %.6f)", 
                       packet.x, packet.y);
            RCLCPP_INFO(this->get_logger(), "  速度: vx=%.6f vy=%.6f w=%.6f",
                       packet.chassis_vx, packet.chassis_vy, packet.chassis_w);
            return;
        }
        
        double dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;
        
        // 2. dt 保护
        const double MIN_DT = 0.0001;  // 0.1ms
        const double MAX_DT = 0.5;     // 500ms
        
        if (dt < MIN_DT) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "dt过小 (%.6fs)，跳过本帧", dt);
            return;
        }
        
        if (dt > MAX_DT) {
            RCLCPP_WARN(this->get_logger(), 
                       "dt过大 (%.3fs)，可能丢包/暂停，重置基准", dt);
            // 重置下位机位移基准
            last_board_x_ = packet.x;
            last_board_y_ = packet.y;
            last_board_pos_valid_ = true;
            return;
        }
        
        // 3. 计算下位机位移增量（机器人坐标系）
        if (!last_board_pos_valid_) {
            last_board_x_ = packet.x;
            last_board_y_ = packet.y;
            last_board_pos_valid_ = true;
            return;
        }
        
        double dx_robot = packet.x - last_board_x_;
        double dy_robot = packet.y - last_board_y_;
        
        // 📊 增强诊断: 打印下位机速度 vs 累积位移的关系
        static int raw_data_counter = 0;
        if (++raw_data_counter >= 10) {
            raw_data_counter = 0;
            
            // 计算理论位移(速度积分)
            double expected_dx = packet.chassis_vx * dt;
            double expected_dy = packet.chassis_vy * dt;
            double expected_disp = std::sqrt(expected_dx*expected_dx + expected_dy*expected_dy);
            double actual_disp = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);
            
            // RCLCPP_INFO(this->get_logger(),
            //     "[诊断] dt=%.3fs | 下位机累积Δ: (%.1f,%.1f)mm 共%.1fmm | "
            //     "速度: vx=%.3f vy=%.3f w=%.3f | 速度积分预期: %.1fmm | 差异: %.1fmm",
            //     dt, dx_robot*1000, dy_robot*1000, actual_disp*1000,
            //     packet.chassis_vx, packet.chassis_vy, packet.chassis_w,
            //     expected_disp*1000, (actual_disp - expected_disp)*1000);
        }
        
        // ⚠️ 检测下位机累积值异常跳变（重置/溢出）
        double displacement_magnitude = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot);
        const double MAX_REASONABLE_DISPLACEMENT = 0.5;  // 0.5m (以dt=0.1s, vmax=0.3m/s, 安全系数10倍)
        
        if (displacement_magnitude > MAX_REASONABLE_DISPLACEMENT) {
            RCLCPP_WARN(this->get_logger(),
                "⚠️ 下位机累积值异常跳变: 增量=%.3fm (%.1fcm) | 上次:(%.3f,%.3f) 当前:(%.3f,%.3f) | 重置基准",
                displacement_magnitude, displacement_magnitude*100,
                last_board_x_, last_board_y_, packet.x, packet.y);
            
            // 重置基准,丢弃本帧增量
            last_board_x_ = packet.x;
            last_board_y_ = packet.y;
            
            // 保持当前累积位置不变,避免大跳变
            publishDebugData(packet, 0.0, 0.0, 0.0, dt, true, false, false);
            publishOdometry();
            if (publish_tf_) publishTransform();
            return;
        }
        
        // 更新下位机位移历史(必须更新以保持差分基准同步)
        last_board_x_ = packet.x;
        last_board_y_ = packet.y;
        
        // 4. 读取下位机实时速度（用于对比验证）
        float vx_board = packet.chassis_vx;
        float vy_board = packet.chassis_vy;
        float wz = packet.chassis_w;
        
        // 5. 读取IMU yaw角（用于坐标转换和纯旋转判断）
        double imu_yaw = packet.yaw;  // IMU航向角 (rad)
        
        // 6. 应用IMU漂移补偿（基于运行时间累积）
        double elapsed_time = (current_time - imu_compensation_start_time_).seconds();
        double drift_compensation = elapsed_time * imu_drift_compensation_rate_;
        imu_yaw += drift_compensation;
        
        // 每60秒输出一次补偿信息
        static int compensation_log_counter = 0;
        if (++compensation_log_counter >= 600) {
            compensation_log_counter = 0;
            RCLCPP_INFO(this->get_logger(), 
                       "[IMU补偿] 运行时间: %.1fs | 累计补偿: %.2f° | 原始yaw: %.2f° | 补偿后yaw: %.2f°",
                       elapsed_time, drift_compensation * 180.0 / M_PI,
                       packet.yaw * 180.0 / M_PI, imu_yaw * 180.0 / M_PI);
        }
        
        // 更新IMU yaw历史（用于计算角速度）
        double delta_yaw = 0.0;
        double actual_wz = 0.0;  // 实际角速度(从IMU计算)
        
        if (last_imu_yaw_valid_) {
            delta_yaw = imu_yaw - last_imu_yaw_;
            // 角度归一化
            while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
            while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
            
            // 从IMU yaw变化量计算实际角速度
            actual_wz = delta_yaw / dt;
        }
        
        last_imu_yaw_ = imu_yaw;
        last_imu_yaw_valid_ = true;
        
        // 7. ⚠️ 全向轮特性: 旋转与平移解耦
        // 全向轮可以全向移动,不需要通过旋转来改变移动方向
        // 旋转只是调整工作姿态,假设旋转时不产生xy位移
        // 简化策略: 只要IMU yaw有变化(>0.3°),就清零xy位移
        
        const double ROTATION_THRESHOLD = 0.005;  // 0.005 rad ≈ 0.3°
        bool is_pure_rotation = std::abs(delta_yaw) > ROTATION_THRESHOLD;
        
        if (is_pure_rotation) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🔄 旋转中: Δyaw=%.2f° → 清零xy位移(原%.1fmm)",
                delta_yaw * 180.0 / M_PI, 
                std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot)*1000);
            dx_robot = 0.0;
            dy_robot = 0.0;
        }
        
        // 8. 位移死区过滤（过滤噪声,但仅用于非旋转时）
        if (!is_pure_rotation) {
            const double DISP_THRESHOLD = 0.0001;    // 0.1mm
            if (std::abs(dx_robot) < DISP_THRESHOLD) dx_robot = 0.0;
            if (std::abs(dy_robot) < DISP_THRESHOLD) dy_robot = 0.0;
        }
        
        // 角速度死区过滤(避免静止时的IMU噪声)
        const double ANGULAR_NOISE_THRESHOLD = 0.015;  // ~0.86°/s 用于过滤IMU噪声
        if (!is_pure_rotation && std::abs(wz) < ANGULAR_NOISE_THRESHOLD) {
            wz = 0.0;
        }
        
        // 9. 上位机速度积分模式（旧方法，用于对比）
        double vel_dx_robot = vx_board * dt;
        double vel_dy_robot = vy_board * dt;
        double cos_theta_for_vel = std::cos(velocity_integrated_theta_);
        double sin_theta_for_vel = std::sin(velocity_integrated_theta_);
        double vel_dx_world = vel_dx_robot * cos_theta_for_vel - vel_dy_robot * sin_theta_for_vel;
        double vel_dy_world = vel_dx_robot * sin_theta_for_vel + vel_dy_robot * cos_theta_for_vel;
        velocity_integrated_x_ += vel_dx_world;
        velocity_integrated_y_ += vel_dy_world;
        
        // 10. 计算综合运动大小（用于静止判断）
        double translation_speed = std::sqrt(dx_robot*dx_robot + dy_robot*dy_robot) / dt;
        double rotation_speed = std::abs(wz) * 0.15;
        double total_motion_speed = translation_speed + rotation_speed;
        
        const double MOTION_THRESHOLD = 0.015; // 15mm/s
        
        if (total_motion_speed < MOTION_THRESHOLD) {
            // 静止：清零速度
            current_vx_ = 0.0;
            current_vy_ = 0.0;
            current_wz_ = 0.0;
            current_vx_robot_ = 0.0;
            current_vy_robot_ = 0.0;
            current_wz_robot_ = 0.0;
            
            // ⚠️ 静止时也要更新角度(可能在原地旋转但速度很慢)
            if (std::abs(delta_yaw) > 0.001) {  // yaw变化 > 0.06°
                current_theta_ += delta_yaw;
                current_theta_ = std::atan2(std::sin(current_theta_), std::cos(current_theta_));
                RCLCPP_DEBUG(this->get_logger(), "静止但在旋转: Δyaw=%.2f°", delta_yaw * 180.0 / M_PI);
            }
            
            // 更新IMU yaw历史
            last_imu_yaw_ = imu_yaw;
            last_imu_yaw_valid_ = true;
            
            publishDebugData(packet, 0.0, 0.0, delta_yaw, dt, true, false, false);
            publishOdometry();
            if (publish_tf_) publishTransform();
            return;
        }
        
        // 11. ⚠️ 先更新角度,再用新角度转换位移
        // 使用步骤7已经计算好的 delta_yaw (从IMU yaw变化量)
        current_theta_ += delta_yaw;
        current_theta_ = std::atan2(std::sin(current_theta_), std::cos(current_theta_));
        
        // 13. ⚠️ 关键: 使用**更新后的current_theta_**转换下位机位移到世界坐标系
        // 下位机的dx/dy是在机器人坐标系,需要用此刻的朝向(current_theta_)转换
        double cos_theta = std::cos(current_theta_);
        double sin_theta = std::sin(current_theta_);
        
        double dx_world = dx_robot * cos_theta - dy_robot * sin_theta;
        double dy_world = dx_robot * sin_theta + dy_robot * cos_theta;
        
        // 14. 累加位姿
        current_x_ += dx_world;
        current_y_ += dy_world;
        
        // 速度积分模式：同样使用IMU角度更新
        velocity_integrated_theta_ += delta_yaw;
        velocity_integrated_theta_ = std::atan2(std::sin(velocity_integrated_theta_), std::cos(velocity_integrated_theta_));
        
        // 📊 定期对比两种积分方式的累积误差
        double position_diff_x = current_x_ - velocity_integrated_x_;
        double position_diff_y = current_y_ - velocity_integrated_y_;
        double position_diff_dist = std::sqrt(position_diff_x*position_diff_x + position_diff_y*position_diff_y);
        double theta_diff = (current_theta_ - velocity_integrated_theta_) * 180.0 / M_PI;
        
        static int compare_log_counter = 0;
        // if (++compare_log_counter >= 50) {  // 每50帧打印一次
        //     compare_log_counter = 0;
        //     RCLCPP_INFO(this->get_logger(),
        //         "[双模式对比] 差分位置:(%.3f, %.3f, %.1f°) | 速度积分:(%.3f, %.3f, %.1f°) | 误差:%.1fmm %.1f°",
        //         current_x_, current_y_, current_theta_*180.0/M_PI,
        //         velocity_integrated_x_, velocity_integrated_y_, velocity_integrated_theta_*180.0/M_PI,
        //         position_diff_dist*1000, theta_diff);
        // }
        
        // 15. 更新速度
        current_vx_ = dx_world / dt;
        current_vy_ = dy_world / dt;
        current_wz_ = actual_wz;  // 使用IMU计算的角速度
        
        // 机器人坐标系速度（用于发布Odometry消息）
        current_vx_robot_ = vx_board;
        current_vy_robot_ = vy_board;
        current_wz_robot_ = wz;
        
        // 16. 发布调试数据
        publishDebugData(packet, dx_world, dy_world, delta_yaw, dt, false, is_pure_rotation, true);
        
        // 17. 发布里程计消息
        publishOdometry();
        
        // 18. 发布TF变换
        if (publish_tf_) {
            publishTransform();
        }
        
        // 调试日志
        RCLCPP_DEBUG(this->get_logger(), 
                    "下位机累积: (%.4f,%.4f) | 增量: dx=%.4f dy=%.4f | IMU方向转换 | "
                    "dt=%.4fs | dθ=%.4f%s | 位姿: x=%.3f y=%.3f θ=%.3f",
                    packet.x, packet.y, dx_robot, dy_robot, dt,
                    delta_yaw, is_pure_rotation ? " [纯旋转]" : "",
                    current_x_, current_y_, current_theta_);
    }
    
    // 发布调试数据（新版：适配速度积分架构 + IMU融合）
    void publishDebugData(const VisionSendPacket& packet, 
                         double dx_world, double dy_world, double dtheta, 
                         double dt, bool is_stationary, 
                         bool is_pure_rotation = false, bool use_imu_rotation = false)
    {
        auto odom_data_msg = std_msgs::msg::Float32MultiArray();
        odom_data_msg.data = {
            packet.chassis_vx,                    // [0] 机器人坐标系速度X (m/s)
            packet.chassis_vy,                    // [1] 机器人坐标系速度Y (m/s)
            packet.chassis_w,                     // [2] 角速度 (rad/s)
            static_cast<float>(dt),               // [3] 时间增量 (s)
            static_cast<float>(dx_world),         // [4] 世界坐标系位移增量X (m)
            static_cast<float>(dy_world),         // [5] 世界坐标系位移增量Y (m)
            static_cast<float>(current_x_),       // [6] 累计位姿X (m)
            static_cast<float>(current_y_),       // [7] 累计位姿Y (m)
            static_cast<float>(current_theta_),   // [8] 累计位姿θ (rad)
            packet.roll,                          // [9] IMU横滚角 (rad)
            packet.pitch,                         // [10] IMU俯仰角 (rad)
            packet.yaw,                           // [11] IMU航向角 (rad)
            static_cast<float>(dtheta),           // [12] 角度增量 (rad)
            static_cast<float>(current_vx_),      // [13] 世界坐标系速度X (m/s)
            static_cast<float>(current_vy_),      // [14] 世界坐标系速度Y (m/s)
            static_cast<float>(is_stationary ? 1.0f : 0.0f)  // [15] 静止标志
        };
        odom_data_pub_->publish(odom_data_msg);
    }
    
    // 发布里程计消息
    void publishOdometry()
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
        
        // 位置 = 当前累积位置
        odom_msg.pose.pose.position.x = current_x_;
        odom_msg.pose.pose.position.y = current_y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        // 姿态 (theta → 四元数)
        tf2::Quaternion q;
        q.setRPY(0, 0, current_theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // 速度 (机器人坐标系 - 符合 nav_msgs/Odometry 标准)
        // REP 105: twist 应该在 child_frame_id (base_link) 坐标系中
        odom_msg.twist.twist.linear.x = current_vx_robot_;
        odom_msg.twist.twist.linear.y = current_vy_robot_;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = current_wz_robot_;
        
        // 协方差矩阵 (根据实测精度调整)
        // 对角线: x, y, z, roll, pitch, yaw
        // 实测：xy 精度 ~1cm，下位机速度很准
        odom_msg.pose.covariance[0] = 0.0001;   // x variance (1cm)² = 0.0001 m²
        odom_msg.pose.covariance[7] = 0.0001;   // y variance (1cm)² = 0.0001 m²
        odom_msg.pose.covariance[14] = 1e6;     // z variance (固定为0，不使用)
        odom_msg.pose.covariance[21] = 1e6;     // roll variance (固定为0，不使用)
        odom_msg.pose.covariance[28] = 1e6;     // pitch variance (固定为0，不使用)
        odom_msg.pose.covariance[35] = 0.1;     // yaw variance (保守估计 ~18°)
        
        // 速度协方差（下位机发送很准确）
        odom_msg.twist.covariance[0] = 0.0001;   // vx variance (很准)
        odom_msg.twist.covariance[7] = 0.0001;   // vy variance (很准)
        odom_msg.twist.covariance[14] = 1e6;     // vz (不使用)
        odom_msg.twist.covariance[21] = 1e6;     // wx (不使用)
        odom_msg.twist.covariance[28] = 1e6;     // wy (不使用)
        odom_msg.twist.covariance[35] = 0.01;    // wz variance (角速度，使用IMU融合后较准)
        
        odom_pub_->publish(odom_msg);
    }
    
    // 发布TF变换 odom → base_link
    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_frame_;
        
        t.transform.translation.x = current_x_;
        t.transform.translation.y = current_y_;
        t.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, current_theta_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);
    }
    
    // 从SLAM获取校正并直接修改里程计位姿
    void correctFromSlam()
    {
        if (!enable_slam_correction_) return;
        
        try {
            // 获取 map -> base_link 的变换（SLAM校正后的真实位姿）
            geometry_msgs::msg::TransformStamped map_to_base;
            map_to_base = tf_buffer_->lookupTransform(
                "map", base_frame_, tf2::TimePointZero);
            
            // 提取位置和角度
            double slam_x = map_to_base.transform.translation.x;
            double slam_y = map_to_base.transform.translation.y;
            
            tf2::Quaternion q(
                map_to_base.transform.rotation.x,
                map_to_base.transform.rotation.y,
                map_to_base.transform.rotation.z,
                map_to_base.transform.rotation.w);
            double roll, pitch, slam_theta;
            tf2::Matrix3x3(q).getRPY(roll, pitch, slam_theta);
            
            // 计算误差
            double error_x = slam_x - current_x_;
            double error_y = slam_y - current_y_;
            double error_theta = slam_theta - current_theta_;
            
            // 角度归一化
            while (error_theta > M_PI) error_theta -= 2.0 * M_PI;
            while (error_theta < -M_PI) error_theta += 2.0 * M_PI;
            
            double error_dist = std::sqrt(error_x * error_x + error_y * error_y);
            double error_angle_deg = std::abs(error_theta * 180.0 / M_PI);
            
            // 根据运动状态动态调整校正增益
            double current_speed = std::sqrt(current_vx_*current_vx_ + current_vy_*current_vy_);
            double correction_gain;
            const char* mode_str;
            
            if (current_speed < slam_static_threshold_) {
                // 静止状态: 默认不校正，信任里程计位置
                // 只有误差较大时才进行一次性校正，避免SLAM噪声引起的位置漂移
                if (error_dist < slam_large_error_threshold_) {
                    RCLCPP_DEBUG(this->get_logger(), "静止且误差小(<%.0fcm),完全信任里程计", slam_large_error_threshold_*100);
                    return;  // 误差小就不动
                }
                
                // 误差较大，进行一次性中等强度校正
                correction_gain = slam_large_error_gain_;
                mode_str = "静止[大误差]";
            } else {
                // 运动状态: 跳过校正，信任高频里程计
                RCLCPP_DEBUG(this->get_logger(), "运动中,跳过SLAM校正 (信任里程计)");
                return;
            }
            
            // 只要有误差就持续校正
            if (error_dist > 0.001 || error_angle_deg > 0.1) {
                // 直接修改里程计位置(⚠️ 会重置下位机差分基准!)
                current_x_ += error_x * correction_gain;
                current_y_ += error_y * correction_gain;
                current_theta_ += error_theta * correction_gain;
                
                // 角度归一化
                current_theta_ = std::atan2(std::sin(current_theta_), std::cos(current_theta_));
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "🔄 SLAM校正[%s] 速度:%.2fm/s | 误差:%.1fcm/%.1f° | 增益:%.0f%% → 校正量:%.1fcm/%.1f°",
                    mode_str, current_speed,
                    error_dist * 100, error_angle_deg,
                    correction_gain * 100,
                    error_dist * correction_gain * 100, error_angle_deg * correction_gain);
            }
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "无法获取SLAM校正: %s", ex.what());
        }
    }
    
    // 重置里程计
    void resetOdometry()
    {
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;
        current_vx_ = 0.0;
        current_vy_ = 0.0;
        current_wz_ = 0.0;
        velocity_integrated_x_ = 0.0;
        velocity_integrated_y_ = 0.0;
        velocity_integrated_theta_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "里程计已重置");
    }
    
    // 重置里程计服务回调
    void resetOdometryCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request;
        (void)response;
        resetOdometry();
    }
    
    // 发布统计信息
    void publishStats()
    {
        RCLCPP_INFO(this->get_logger(), 
                   "统计: 收包=%ld, 无效=%ld, CRC错误=%ld | 位姿: (%.2f, %.2f, %.2f°)",
                   packets_received_, packets_invalid_, packets_crc_error_,
                   current_x_, current_y_, current_theta_ * 180.0 / M_PI);
    }
    
    // ROS接口
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_rx_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr odom_data_pub_;  // 调试用
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_srv_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr slam_correction_timer_;
    
    // 参数
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;
    bool enable_crc_check_;
    double imu_drift_compensation_rate_;  // IMU漂移补偿速率 (rad/s)
    bool enable_slam_correction_;         // 是否启用SLAM校正
    double slam_correction_interval_;     // SLAM校正间隔 (s)
    double slam_static_threshold_;        // 静止判定阈值 (m/s)
    double slam_moving_gain_;             // 运动时校正增益 (0-1)
    double slam_static_gain_;             // 静止时校正增益 (0-1)
    double slam_large_error_threshold_;   // 大误差阈值 (m) - 触发强校正
    double slam_large_error_gain_;        // 大误差校正增益 (0-1)
    rclcpp::Time imu_compensation_start_time_{0, 0, RCL_ROS_TIME};  // 补偿计时起点
    
    // 当前位姿 (世界坐标系 - odom frame)
    double current_x_{0.0};
    double current_y_{0.0};
    double current_theta_{0.0};
    
    // 当前速度 (世界坐标系 - 用于调试)
    double current_vx_{0.0};
    double current_vy_{0.0};
    double current_wz_{0.0};
    
    // 当前速度 (机器人坐标系 - 用于发布Odometry)
    double current_vx_robot_{0.0};
    double current_vy_robot_{0.0};
    double current_wz_robot_{0.0};
    
    // 时间基准 (ROS时间)
    rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
    
    // IMU yaw角历史（用于计算角速度）
    double last_imu_yaw_{0.0};
    bool last_imu_yaw_valid_{false};
    
    // 下位机累积位移历史（用于计算增量）
    double last_board_x_{0.0};
    double last_board_y_{0.0};
    bool last_board_pos_valid_{false};
    
    // 速度积分累积位置（用于对比验证）
    double velocity_integrated_x_{0.0};
    double velocity_integrated_y_{0.0};
    double velocity_integrated_theta_{0.0};
    
    // 统计
    long packets_received_{0};
    long packets_invalid_{0};
    long packets_crc_error_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
