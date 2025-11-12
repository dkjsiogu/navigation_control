/**
 * @file scan_filter_node.cpp
 * @brief 激光扫描过滤节点 - 过滤机器人本体点云
 * 
 * 功能：
 * 1. 订阅原始 /scan 话题
 * 2. 过滤指定角度范围内的点云（机器人本体）
 * 3. 发布过滤后的 /scan_filtered 话题
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class ScanFilterNode : public rclcpp::Node
{
public:
    ScanFilterNode() : Node("scan_filter_node")
    {
        // 声明参数 - 角度+距离组合过滤
        // 雷达倒装(X朝后Y朝右)，过滤机器人后方本体
        this->declare_parameter<double>("filter_angle_min", -2.30);  // -132° (左后角)
        this->declare_parameter<double>("filter_angle_max", 2.69);   // 154° (右后角)
        this->declare_parameter<double>("filter_range_max", 0.35);   // 只过滤此距离内的点
        this->declare_parameter<std::string>("input_topic", "/scan");
        this->declare_parameter<std::string>("output_topic", "/scan_filtered");
        
        // 获取参数
        filter_angle_min_ = this->get_parameter("filter_angle_min").as_double();
        filter_angle_max_ = this->get_parameter("filter_angle_max").as_double();
        filter_range_max_ = this->get_parameter("filter_range_max").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        // 创建订阅和发布
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic, 10,
            std::bind(&ScanFilterNode::scanCallback, this, std::placeholders::_1));
        
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "激光扫描过滤节点已启动 (角度+距离组合)");
        RCLCPP_INFO(this->get_logger(), "  输入: %s -> 输出: %s", 
                    input_topic.c_str(), output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  过滤角度: %.2f° 到 %.2f°",
                    filter_angle_min_ * 180.0 / M_PI,
                    filter_angle_max_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  仅过滤距离 < %.2f m 的点 (机器人本体)",
                    filter_range_max_);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        
        int filtered_count = 0;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            // 计算当前点的角度和距离
            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];
            
            // 规范化角度到 [-π, π]
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            
            // 检查是否在过滤角度范围内
            bool in_filter_angle = false;
            if (filter_angle_min_ <= filter_angle_max_) {
                // 正常范围
                in_filter_angle = (angle >= filter_angle_min_ && angle <= filter_angle_max_);
            } else {
                // 跨越 ±π 的范围
                in_filter_angle = (angle >= filter_angle_min_ || angle <= filter_angle_max_);
            }
            
            // 只过滤：在机器人本体角度范围内 且 距离小于阈值的点
            // 远处的点即使在机器人方向也保留
            if (in_filter_angle && !std::isinf(range) && range < filter_range_max_) {
                filtered_msg->ranges[i] = std::numeric_limits<float>::infinity();
                filtered_count++;
            }
        }
        
        scan_pub_->publish(*filtered_msg);
        
        // 定期输出统计信息
        static int count = 0;
        if (++count % 50 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "过滤了 %d/%lu 个点 (机器人本体范围内)", 
                        filtered_count, msg->ranges.size());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    
    double filter_angle_min_;
    double filter_angle_max_;
    double filter_range_max_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
