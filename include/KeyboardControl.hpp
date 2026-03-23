#ifndef KEYBOARDCONTROL_HPP
#define KEYBOARDCONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


class KeyboardControlNode : public rclcpp::Node {
public:
    KeyboardControlNode();

    ~KeyboardControlNode();

private:

    void timerCallback();

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    struct termios old_termios_;

    geometry_msgs::msg::Twist current_twist_;
    
    int no_key_timeout_ = 0;

    double current_speed_ = 0.5;
};

#endif // KEYBOARDCONTROL_HPP
