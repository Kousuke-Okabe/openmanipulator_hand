#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hand_control_interfaces/msg/move_hand.hpp"

using namespace std::chrono_literals;

class HandControlTestPublisher : public rclcpp::Node
{
public:
    HandControlTestPublisher() : Node("hand_control_test_publisher"), is_open_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Hand Control Test Publisher Node started");
        
        // QoS設定
        this->declare_parameter("qos_depth", 10);
        int8_t qos_depth = 0;
        this->get_parameter("qos_depth", qos_depth);
        
        const auto QOS_RKL10V =
            rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
        
        // パブリッシャーの作成
        publisher_ = this->create_publisher<hand_control_interfaces::msg::MoveHand>(
            "Hand_control", QOS_RKL10V);
        
        // 5秒間隔のタイマー作成
        timer_ = this->create_wall_timer(
            5000ms, std::bind(&HandControlTestPublisher::timer_callback, this));
        
        // Dynamixel IDの設定（必要に応じて変更）
        this->declare_parameter("dynamixel_id", 1);
        this->get_parameter("dynamixel_id", dynamixel_id_);
        
        RCLCPP_INFO(this->get_logger(), "Publishing to Hand_control topic every 5 seconds");
        RCLCPP_INFO(this->get_logger(), "Using Dynamixel ID: %d", dynamixel_id_);
        
        // 初回メッセージ送信
        send_hand_command();
    }

private:
    void timer_callback()
    {
        send_hand_command();
    }
    
    void send_hand_command()
    {
        auto message = hand_control_interfaces::msg::MoveHand();
        
        // IDを設定
        message.id = dynamixel_id_;
        
        // 開閉状態を切り替え
        if (is_open_) {
            message.state = 'C';  // Close
            RCLCPP_INFO(this->get_logger(), "Sending CLOSE command [ID: %d]", message.id);
        } else {
            message.state = 'O';  // Open
            RCLCPP_INFO(this->get_logger(), "Sending OPEN command [ID: %d]", message.id);
        }
        
        // メッセージを送信
        publisher_->publish(message);
        
        // 状態を切り替え
        is_open_ = !is_open_;
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<hand_control_interfaces::msg::MoveHand>::SharedPtr publisher_;
    bool is_open_;
    int dynamixel_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<HandControlTestPublisher>();
    
    RCLCPP_INFO(node->get_logger(), "Hand Control Test Publisher is running...");
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to stop");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", e.what());
    }
    
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Hand Control Test Publisher stopped");
    
    return 0;
}