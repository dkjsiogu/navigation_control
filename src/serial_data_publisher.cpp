/**
 * @file serial_data_publisher.cpp
 * @brief 全向轮控制 - 串口数据发布器
 * 
 * 功能:
 * 1. 订阅速度命令 (/cmd_vel)
 * 2. 订阅颜色跟踪结果 (/color_tracking/result)
 * 3. 转换为下位机 Vision_Recv_s 协议并通过串口发送
 * 
 * 协议格式 (31字节) - 匹配下位机 Vision_Recv_s 短包:
 * - header: 0xA5 (1字节)
 * - x: float (4字节) - dx 目标偏转角度 (rad)
 * - y: float (4字节) - dy 目标垂直偏移 (pixels)
 * - z: float (4字节) - flog 颜色跟踪有效标志
 * - vx: float (4字节) - 前后速度 (m/s)
 * - vy: float (4字节) - 左右速度 (m/s)
 * - vz: float (4字节) - 旋转速度 (rad/s)
 * - cap_timestamp: uint32_t (4字节) - 时间戳 (ms)
 * - checksum: uint16_t (2字节) - CRC16校验
 * 
 * 下位机读取映射:
 * - Vision_Info.dx = recv_data.x
 * - Vision_Info.dy = recv_data.y
 * - Vision_Info.flog = recv_data.z
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstring>

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

// CRC16计算函数 (与下位机完全一致)
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

// 全向轮控制协议 - 31字节短包格式（匹配下位机 DecodeVision）
// 布局: header(1) + x(4) + y(4) + z(4) + vx(4) + vy(4) + wz(4) + timestamp(4) + CRC16(2) = 31字节
struct __attribute__((packed)) OmniWheelCmd {
    uint8_t header;        // 0xA5
    float x;               // dx - 目标偏转角度 (rad)
    float y;               // dy - 目标垂直偏移 (pixels)  
    float z;               // flog - 颜色跟踪有效标志
    float vx;              // 前后速度 (m/s)
    float vy;              // 左右速度 (m/s)
    float vz;              // 旋转速度 (rad/s) - 下位机读取为 wz
    uint32_t cap_timestamp; // 时间戳 (ms)
    uint16_t checksum;     // CRC16校验
    
    // 计算CRC16
    void calculateChecksum() {
        checksum = Get_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t*>(this), 
            sizeof(OmniWheelCmd) - 2, 
            CRC16_INIT
        );
    }
    
    // 转换为字节数组
    std::vector<uint8_t> toBytes() {
        calculateChecksum();
        std::vector<uint8_t> bytes(sizeof(OmniWheelCmd));
        memcpy(bytes.data(), this, sizeof(OmniWheelCmd));
        return bytes;
    }
};

class SerialDataPublisher : public rclcpp::Node
{
public:
    SerialDataPublisher() : Node("serial_data_publisher")
    {
        // 声明参数
        this->declare_parameter("max_vx", 1.0);           // 最大前后速度 m/s
        this->declare_parameter("max_vy", 1.0);           // 最大左右速度 m/s
        this->declare_parameter("max_wz", 2.0);           // 最大旋转速度 rad/s
        this->declare_parameter("velocity_timeout", 1.0); // 速度超时 s
        this->declare_parameter("smooth_factor", 0.7);    // 平滑因子 0-1
        
        max_vx_ = this->get_parameter("max_vx").as_double();
        max_vy_ = this->get_parameter("max_vy").as_double();
        max_wz_ = this->get_parameter("max_wz").as_double();
        velocity_timeout_ = this->get_parameter("velocity_timeout").as_double();
        smooth_factor_ = this->get_parameter("smooth_factor").as_double();
        
        // 订阅速度命令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialDataPublisher::cmdVelCallback, this, std::placeholders::_1));
        
        // 订阅颜色跟踪结果 (get-toy 发布的)
        color_tracking_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/color_tracking/result", 10,
            std::bind(&SerialDataPublisher::colorTrackingCallback, this, std::placeholders::_1));
        
        // 发布串口数据
        serial_data_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "serial_tx_data", 10);
        
        // 发布调试信息
        serial_hex_pub_ = this->create_publisher<std_msgs::msg::String>(
            "serial_tx_hex", 10);
        
        // 定时器 - 检查超时
        timeout_timer_ = this->create_wall_timer(
            100ms, std::bind(&SerialDataPublisher::checkTimeout, this));
        
        // 定时器 - 持续发送数据包 (50Hz)
        send_timer_ = this->create_wall_timer(
            20ms, std::bind(&SerialDataPublisher::sendTimerCallback, this));
        
        // 初始化时间戳
        last_cmd_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "全向轮控制节点已初始化");
        RCLCPP_INFO(this->get_logger(), "最大速度 - Vx: %.2f m/s, Vy: %.2f m/s, Wz: %.2f rad/s", 
                    max_vx_, max_vy_, max_wz_);
        RCLCPP_INFO(this->get_logger(), "数据发送频率: 50 Hz");
    }

private:
    // 颜色跟踪结果回调
    void colorTrackingCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 解析统一格式: "status,rad,y_offset"
        // status: 0.00=无效 1.00=彩色跟踪 2.00=黑色检测
        std::istringstream iss(msg->data);
        std::string status_str, rad_str, dy_str;
        
        if (std::getline(iss, status_str, ',') &&
            std::getline(iss, rad_str, ',') &&
            std::getline(iss, dy_str)) {
            
            try {
                float status = std::stof(status_str);
                tracking_status_ = status;  // 保存原始状态值
                tracking_valid_ = (status > 0.5f);  // > 0.5 认为有效（1或2都有效）
                tracking_rad_ = std::stod(rad_str);
                tracking_dy_ = static_cast<int>(std::round(std::stof(dy_str)));
                
                // 根据状态值显示不同模式
                const char* mode_name = (status > 1.5f) ? "黑色检测" : 
                                       (status > 0.5f) ? "彩色跟踪" : "无效";
                RCLCPP_DEBUG(this->get_logger(), "视觉[%s]: status=%.2f valid=%d rad=%.3f dy=%d", 
                            mode_name, status, tracking_valid_, tracking_rad_, tracking_dy_);
            } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "解析视觉数据失败: %s", e.what());
                tracking_valid_ = false;
                tracking_status_ = 0.0f;
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "视觉数据格式错误: %s", msg->data.c_str());
            tracking_valid_ = false;
            tracking_status_ = 0.0f;
        }
    }
    
    // 速度命令回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();
        
        // 获取速度
        double vx = msg->linear.x;   // 前后
        double vy = msg->linear.y;   // 左右
        double wz = msg->angular.z;  // 旋转
        
        // 限幅
        vx = std::clamp(vx, -max_vx_, max_vx_);
        vy = std::clamp(vy, -max_vy_, max_vy_);
        wz = std::clamp(wz, -max_wz_, max_wz_);
        
        // 平滑滤波（不立即发送，由定时器统一发送）
        smoothed_vx_ = smooth_factor_ * smoothed_vx_ + (1.0 - smooth_factor_) * vx;
        smoothed_vy_ = smooth_factor_ * smoothed_vy_ + (1.0 - smooth_factor_) * vy;
        smoothed_wz_ = smooth_factor_ * smoothed_wz_ + (1.0 - smooth_factor_) * wz;
    }
    
    // 定时发送回调 - 50Hz 持续发送
    void sendTimerCallback()
    {
        sendSerialData(smoothed_vx_, smoothed_vy_, smoothed_wz_);
    }
    
    // 发送串口数据
    void sendSerialData(double vx, double vy, double wz)
    {
        OmniWheelCmd packet;
        packet.header = 0xA5;
        
        // 视觉数据映射（匹配下位机 Vision_Info 读取）
        // 下位机: Vision_Info.dx = recv_data.x, dy = recv_data.y, flog = recv_data.z
        packet.x = static_cast<float>(tracking_rad_);   // dx - 角度 (rad)
        packet.y = static_cast<float>(tracking_dy_);    // dy - 垂直偏移 (pixels)
        packet.z = tracking_status_;                    // flog - 状态标志 (0=无效 1=彩色 2=黑色)
        
        // 速度控制
        packet.vx = static_cast<float>(vx);
        packet.vy = static_cast<float>(vy);
        packet.vz = static_cast<float>(wz);
        
        packet.cap_timestamp = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
        
    // 调试输出：显示发送的所有字段值（节流，避免日志刷屏）
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //            "发送[31B] -> x(dx):%.3f y(dy):%.0f z(flog):%.1f | vx:%.2f vy:%.2f vz:%.2f",
    //            packet.x, packet.y, packet.z, packet.vx, packet.vy, packet.vz);
        
        auto bytes = packet.toBytes();
        
        // 发布字节数据
        auto data_msg = std_msgs::msg::UInt8MultiArray();
        data_msg.data = bytes;
        serial_data_pub_->publish(data_msg);
        
        // 发布十六进制 (调试)
        std::stringstream hex;
        hex << "TX[" << bytes.size() << "]: ";
        for (uint8_t b : bytes) {
            hex << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        }
        
        auto hex_msg = std_msgs::msg::String();
        hex_msg.data = hex.str();
        serial_hex_pub_->publish(hex_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "发送: vx=%.2f vy=%.2f wz=%.2f", vx, vy, wz);
    }
    
    // 超时检查
    void checkTimeout()
    {
        auto elapsed = (this->now() - last_cmd_time_).seconds();
        
        if (elapsed > velocity_timeout_ && 
            (std::abs(smoothed_vx_) > 0.01 || std::abs(smoothed_vy_) > 0.01 || std::abs(smoothed_wz_) > 0.01)) {
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "速度命令超时 (%.1fs)，发送停止命令", elapsed);
            
            smoothed_vx_ = 0.0;
            smoothed_vy_ = 0.0;
            smoothed_wz_ = 0.0;
            sendSerialData(0.0, 0.0, 0.0);
        }
    }
    
    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_tracking_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_data_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_hex_pub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr send_timer_;  // 50Hz 定时发送
    
    double max_vx_;
    double max_vy_;
    double max_wz_;
    double velocity_timeout_;
    double smooth_factor_;
    
    double smoothed_vx_{0.0};
    double smoothed_vy_{0.0};
    double smoothed_wz_{0.0};
    
    rclcpp::Time last_cmd_time_;
    
    // 视觉跟踪数据
    bool tracking_valid_{false};
    float tracking_status_{0.0f};  // 状态值: 0=无效 1=彩色跟踪 2=黑色检测
    double tracking_rad_{0.0};
    int tracking_dy_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialDataPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
