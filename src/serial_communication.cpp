/**
 * @file serial_communication.cpp
 * @brief 串口通信节点 - 与开发板进行实际串口通信
 * 
 * 功能:
 * 1. 订阅serial_tx_data，通过串口发送给下位机
 * 2. 从串口接收下位机数据（里程计、状态等）
 * 3. 发布接收到的数据
 * 4. 自动重连和错误处理
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>
#include <thread>
#include <vector>
#include <sstream>
#include <iomanip>
#include <memory>

using namespace LibSerial;

using namespace std::chrono_literals;
using namespace LibSerial;

class SerialCommunication : public rclcpp::Node
{
public:
    SerialCommunication() : Node("serial_communication")
    {
        // 声明参数
        this->declare_parameter("serial_port", "/dev/ttyACM0");  // Micro USB通常是ttyACM0
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("timeout_ms", 100);
        this->declare_parameter("auto_reconnect", true);
        this->declare_parameter("reconnect_interval", 5.0);  // 重连间隔(秒)
        
        serial_port_ = this->get_parameter("serial_port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();
        auto_reconnect_ = this->get_parameter("auto_reconnect").as_bool();
        reconnect_interval_ = this->get_parameter("reconnect_interval").as_double();
        
        // 订阅要发送的串口数据
        serial_tx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "serial_tx_data", 10,
            std::bind(&SerialCommunication::serialTxCallback, this, std::placeholders::_1));
        
        // 发布接收到的原始数据
        serial_rx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "serial_rx_data", 10);
        
        // 发布接收到的十六进制字符串
        serial_rx_hex_pub_ = this->create_publisher<std_msgs::msg::String>(
            "serial_rx_hex", 10);
        
        // 发布连接状态
        connection_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "serial_connection_status", 10);
        
        // 定时器 - 检查连接状态
        status_timer_ = this->create_wall_timer(
            1s, std::bind(&SerialCommunication::publishConnectionStatus, this));
        
        // 定时器 - 读取串口数据
        read_timer_ = this->create_wall_timer(
            10ms, std::bind(&SerialCommunication::readSerialData, this));
        
        // 定时器 - 自动重连
        if (auto_reconnect_) {
            reconnect_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(reconnect_interval_),
                std::bind(&SerialCommunication::tryReconnect, this));
        }
        
        // RCLCPP_INFO(this->get_logger(), "Serial Communication Node initialized");
        // RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
        // RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate_);
        
        // 尝试打开串口
        openSerialPort();
    }
    
    ~SerialCommunication()
    {
        closeSerialPort();
    }

private:
    // 打开串口
    bool openSerialPort()
    {
        try {
            if (serial_port_obj_.IsOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port already open");
                return true;
            }
            
            RCLCPP_INFO(this->get_logger(), "Opening serial port: %s", serial_port_.c_str());
            
            serial_port_obj_.Open(serial_port_);
            
            // 设置波特率
            BaudRate baud;
            switch(baudrate_) {
                case 9600:   baud = BaudRate::BAUD_9600; break;
                case 19200:  baud = BaudRate::BAUD_19200; break;
                case 38400:  baud = BaudRate::BAUD_38400; break;
                case 57600:  baud = BaudRate::BAUD_57600; break;
                case 115200: baud = BaudRate::BAUD_115200; break;
                case 230400: baud = BaudRate::BAUD_230400; break;
                case 460800: baud = BaudRate::BAUD_460800; break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unsupported baudrate: %d", baudrate_);
                    return false;
            }
            serial_port_obj_.SetBaudRate(baud);
            
            // 设置字符大小
            serial_port_obj_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            
            // 设置奇偶校验
            serial_port_obj_.SetParity(Parity::PARITY_NONE);
            
            // 设置停止位
            serial_port_obj_.SetStopBits(StopBits::STOP_BITS_1);
            
            // 设置流控制
            serial_port_obj_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            
            is_connected_ = true;
            connection_lost_count_ = 0;
            
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully!");
            RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d, 8N1", 
                       serial_port_.c_str(), baudrate_);
            RCLCPP_INFO(this->get_logger(), "Waiting for data from STM32...");
            
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            is_connected_ = false;
            return false;
        }
    }
    
    // 关闭串口
    void closeSerialPort()
    {
        try {
            if (serial_port_obj_.IsOpen()) {
                serial_port_obj_.Close();
                RCLCPP_INFO(this->get_logger(), "Serial port closed");
            }
            is_connected_ = false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error closing serial port: %s", e.what());
        }
    }
    
    // 发送数据回调
    void serialTxCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!is_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Serial port not connected, cannot send data");
            return;
        }
        
        try {
            // 发送数据
            serial_port_obj_.Write(msg->data);
            serial_port_obj_.DrainWriteBuffer();  // 确保数据发送完成
            
            bytes_sent_ += msg->data.size();
            packets_sent_++;
            
            //RCLCPP_DEBUG(this->get_logger(), "Sent %zu bytes to serial port", msg->data.size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending data: %s", e.what());
            is_connected_ = false;
            connection_lost_count_++;
        }
    }
    
    // 读取串口数据
    void readSerialData()
    {
        if (!is_connected_ || !serial_port_obj_.IsOpen()) {
            return;
        }
        
        try {
            std::vector<uint8_t> buffer;
            
            // 一次性读取所有可用数据
            while (serial_port_obj_.IsDataAvailable()) {
                char byte;
                try {
                    serial_port_obj_.ReadByte(byte, timeout_ms_);
                    buffer.push_back(static_cast<uint8_t>(byte));
                } catch (const ReadTimeout&) {
                    // 超时正常,跳出循环
                    break;
                }
            }
            
            if (!buffer.empty()) {
                bytes_received_ += buffer.size();
                packets_received_++;
                
                // 发布原始数据
                auto rx_msg = std_msgs::msg::UInt8MultiArray();
                rx_msg.data = buffer;
                serial_rx_pub_->publish(rx_msg);
                
                // 发布十六进制字符串
                std::stringstream hex_stream;
                hex_stream << "RX: ";
                for (uint8_t byte : buffer) {
                    hex_stream << std::hex << std::setw(2) << std::setfill('0') 
                              << static_cast<int>(byte) << " ";
                }
                
                auto hex_msg = std_msgs::msg::String();
                hex_msg.data = hex_stream.str();
                serial_rx_hex_pub_->publish(hex_msg);
                
                // 改为DEBUG级别，避免日志刷屏
                //RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes: %s", 
                //            buffer.size(), hex_msg.data.c_str());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Error reading data: %s", e.what());
            is_connected_ = false;
            connection_lost_count_++;
        }
    }
    
    // 尝试重连
    void tryReconnect()
    {
        if (!is_connected_ && auto_reconnect_) {
            RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to serial port...");
            closeSerialPort();
            openSerialPort();
        }
    }
    
    // 发布连接状态
    void publishConnectionStatus()
    {
        auto msg = std_msgs::msg::String();
        
        if (is_connected_) {
            msg.data = "CONNECTED - Port: " + serial_port_ + 
                      ", TX: " + std::to_string(packets_sent_) + " packets (" + 
                      std::to_string(bytes_sent_) + " bytes), " +
                      "RX: " + std::to_string(packets_received_) + " packets (" + 
                      std::to_string(bytes_received_) + " bytes)";
            
            // 如果长时间没有接收到数据,给出提示
            if (packets_received_ == 0) {
                msg.data += " [WARNING: No data received from STM32]";
            }
        } else {
            msg.data = "DISCONNECTED - Lost: " + std::to_string(connection_lost_count_) + " times";
        }
        
        connection_status_pub_->publish(msg);
        
        // 打印到控制台
        //RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
    }
    
    // 成员变量
    SerialPort serial_port_obj_;
    
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_tx_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_rx_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_rx_hex_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr connection_status_pub_;
    
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    
    std::string serial_port_;
    int baudrate_;
    int timeout_ms_;
    bool auto_reconnect_;
    double reconnect_interval_;
    
    bool is_connected_{false};
    int connection_lost_count_{0};
    size_t bytes_sent_{0};
    size_t bytes_received_{0};
    int packets_sent_{0};
    int packets_received_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommunication>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
