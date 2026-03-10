#include <cstdio>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class StudentPublisher : public rclcpp::Node
{
public:
  StudentPublisher() : Node("student_node")
  {
    this->declare_parameter<float>("max_voltage", 42.0);
    this->declare_parameter<float>("min_voltage", 32.0);
    // Create a publisher 
    publisher_name_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    subscriber_battery_voltage_ = this->create_subscription<std_msgs::msg::Float32>("battery_voltage", 10, 
      std::bind(&StudentPublisher::battery_voltage_callback, this, std::placeholders::_1));
    publisher_battery_percentage_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
    // Timer for periodic publishing
    timer_ = this->create_wall_timer(
      500ms, std::bind(&StudentPublisher::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_name_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_battery_voltage_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_battery_percentage_;

  void battery_voltage_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    double max_v = this->get_parameter("max_voltage").as_double();
    double min_v = this->get_parameter("min_voltage").as_double();

    float voltage = msg->data;
    float percentage = static_cast<float>((voltage - min_v) / (max_v - min_v) * 100.0);
    
    // Publish battery percentage
    auto percentage_msg = std_msgs::msg::Float32();
    percentage_msg.data = percentage;
    publisher_battery_percentage_->publish(percentage_msg);
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = this->get_name();
    publisher_name_->publish(message);
  }
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StudentPublisher>());
  rclcpp::shutdown();
  return 0;
}