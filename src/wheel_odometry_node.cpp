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
#include <tf2/LinearMath/Quaternion.h>
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
        
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        enable_crc_check_ = this->get_parameter("enable_crc_check").as_bool();
        imu_drift_compensation_rate_ = this->get_parameter("imu_drift_compensation_deg_per_min").as_double() 
                                       * (M_PI / 180.0) / 60.0;  // 转换为 rad/s
        
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
        // ===== 架构变更：现在使用实时速度积分，而非预积分位移 =====
        // 使用 chassis_vx/vy/w (机器人坐标系速度) + ROS时间戳计算dt
        
        // 1. 计算时间增量 (使用ROS时间，忽略下位机timestamp)
        rclcpp::Time current_time = this->now();
        
        if (!last_update_time_.nanoseconds()) {
            // 第一帧：只初始化时间，不积分
            last_update_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "里程计初始化：等待下一帧开始积分");
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
                       "dt过大 (%.3fs)，可能丢包/暂停，重置时间基准", dt);
            // 不积分，只更新时间（已在上面更新）
            return;
        }
        
        // 3. 读取机器人坐标系速度（下位机发送的实时值）
        float vx_robot = packet.chassis_vx;  // 前进速度 (m/s)
        float vy_robot = packet.chassis_vy;  // 左侧速度 (m/s)
        float wz = packet.chassis_w;         // 角速度 (rad/s)
        
        // 3.5 读取IMU yaw角（用于后面计算变化量）
        double imu_yaw = packet.yaw;  // IMU航向角 (rad)
        
        // 3.6 应用IMU漂移补偿（基于运行时间累积）
        double elapsed_time = (current_time - imu_compensation_start_time_).seconds();
        double drift_compensation = elapsed_time * imu_drift_compensation_rate_;  // 累计补偿量 (rad)
        imu_yaw += drift_compensation;  // 应用补偿
        
        // 每60秒输出一次补偿信息
        static int compensation_log_counter = 0;
        if (++compensation_log_counter >= 600) {  // 假设数据包频率~10Hz，600次≈60秒
            compensation_log_counter = 0;
            RCLCPP_INFO(this->get_logger(), 
                       "[IMU补偿] 运行时间: %.1fs | 累计补偿: %.2f° | 原始yaw: %.2f° | 补偿后yaw: %.2f°",
                       elapsed_time, drift_compensation * 180.0 / M_PI,
                       packet.yaw * 180.0 / M_PI, imu_yaw * 180.0 / M_PI);
        }
        
        // 4. 速度死区过滤（过滤编码器噪声）
        const double VEL_THRESHOLD = 0.001;    // 1mm/s
        const double ANGULAR_THRESHOLD = 0.001; // ~0.06°/s
        
        if (std::abs(vx_robot) < VEL_THRESHOLD) vx_robot = 0.0f;
        if (std::abs(vy_robot) < VEL_THRESHOLD) vy_robot = 0.0f;
        if (std::abs(wz) < ANGULAR_THRESHOLD) wz = 0.0f;
        
        // 5. 计算综合运动大小（用于静止判断）
        double translation_speed = std::sqrt(vx_robot*vx_robot + vy_robot*vy_robot);
        double rotation_speed = std::abs(wz) * 0.15;  // 乘以特征半径转为线速度等效
        double total_motion_speed = translation_speed + rotation_speed;
        
        const double MOTION_THRESHOLD = 0.002; // 2mm/s 综合运动阈值
        
        if (total_motion_speed < MOTION_THRESHOLD) {
            // 静止：速度指数衰减而非突变为0
            current_vx_ *= 0.7;
            current_vy_ *= 0.7;
            current_wz_ *= 0.7;
            
            // ⚠️ 静止时冻结IMU Yaw更新（防止陀螺仪漂移累积）
            // 更新 last_imu_yaw_ 为当前值，但不累积到 current_theta_
            // 这样下次运动时 delta_yaw 会从当前IMU值开始计算，自动同步
            last_imu_yaw_ = imu_yaw;
            last_imu_yaw_valid_ = true;
            
            publishDebugData(packet, 0.0, 0.0, 0.0, dt, true, false, false);
            publishOdometry();
            if (publish_tf_) publishTransform();
            return;
        }
        
        // 6. 判断是否为纯旋转（yaw有变化但xy不应累积）
        const double PURE_ROTATION_VEL_THRESHOLD = 0.02;  // 2cm/s 平移速度阈值
        const double PURE_ROTATION_ANGULAR_THRESHOLD = 0.05;  // ~2.9°/s 角速度阈值
        
        // 使用之前计算的 translation_speed（避免重复定义）
        bool is_pure_rotation = (std::abs(wz) > PURE_ROTATION_ANGULAR_THRESHOLD) && 
                                (translation_speed < PURE_ROTATION_VEL_THRESHOLD);
        
        // 7. 机器人坐标系速度 -> 位移增量（简单矩形积分）
        double dx_robot = is_pure_rotation ? 0.0 : vx_robot * dt;  // 纯旋转时不累积xy
        double dy_robot = is_pure_rotation ? 0.0 : vy_robot * dt;  // 纯旋转时不累积xy
        
        // 8. 转换到世界坐标系（使用当前角度）
        double cos_theta = std::cos(current_theta_);
        double sin_theta = std::sin(current_theta_);
        
        double dx_world = dx_robot * cos_theta - dy_robot * sin_theta;
        double dy_world = dx_robot * sin_theta + dy_robot * cos_theta;
        
        // 9. 累加位姿
        current_x_ += dx_world;
        current_y_ += dy_world;
        
        // ⚠️ 使用IMU Yaw的变化量来更新累积角度（而非编码器角速度积分），IMU旋转测量更准确
        double delta_yaw_for_pose = 0.0;
        if (last_imu_yaw_valid_) {
            // 计算IMU yaw变化量（处理跨越±π的情况）
            delta_yaw_for_pose = imu_yaw - last_imu_yaw_;
            
            // 角度归一化到 [-π, π]
            while (delta_yaw_for_pose > M_PI) delta_yaw_for_pose -= 2.0 * M_PI;
            while (delta_yaw_for_pose < -M_PI) delta_yaw_for_pose += 2.0 * M_PI;
            
            // 累加角度变化量（而非直接用IMU Yaw绝对值，因为IMU初始角度可能不是0）
            current_theta_ += delta_yaw_for_pose;
            
            // 角度归一化到 [-π, π]
            current_theta_ = std::atan2(std::sin(current_theta_), std::cos(current_theta_));
        }
        
        // 更新IMU yaw历史（为下一帧计算变化量做准备）
        last_imu_yaw_ = imu_yaw;
        last_imu_yaw_valid_ = true;
        
        // 10. 更新速度
        // 世界坐标系速度（用于调试显示）
        current_vx_ = dx_world / dt;
        current_vy_ = dy_world / dt;
        current_wz_ = delta_yaw_for_pose / dt;  // 使用IMU yaw变化量计算角速度
        
        // 机器人坐标系速度（用于发布Odometry消息）
        current_vx_robot_ = vx_robot;
        current_vy_robot_ = vy_robot;
        current_wz_robot_ = wz;
        
        // 11. 发布调试数据
        publishDebugData(packet, dx_world, dy_world, delta_yaw_for_pose, dt, false, is_pure_rotation, true);
        
        // 12. 发布里程计消息
        publishOdometry();
        
        // 13. 发布TF变换
        if (publish_tf_) {
            publishTransform();
        }
        
        // 调试日志（详细版）
        RCLCPP_DEBUG(this->get_logger(), 
                    "速度: vx=%.3f vy=%.3f w=%.3f | dt=%.4fs | "
                    "增量: dx=%.4f dy=%.4f dθ=%.4f(IMU)%s | "
                    "位姿: x=%.3f y=%.3f θ=%.3f",
                    vx_robot, vy_robot, wz, dt,
                    dx_world, dy_world, delta_yaw_for_pose, is_pure_rotation ? " [纯旋转-不累积XY]" : "",
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
        
        // 位置
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
    
    // 发布TF变换
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
    
    // 重置里程计
    void resetOdometry()
    {
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;
        current_vx_ = 0.0;
        current_vy_ = 0.0;
        current_wz_ = 0.0;
        
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
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_srv_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    // 参数
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;
    bool enable_crc_check_;
    double imu_drift_compensation_rate_;  // IMU漂移补偿速率 (rad/s)
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
