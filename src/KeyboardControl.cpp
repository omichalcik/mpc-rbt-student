#include <chrono>
#include <functional>

#include "KeyboardControl.hpp"

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode(): rclcpp::Node("keyboard_control_node") {

    this->declare_parameter<double>("speed", 0.5);

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardControlNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Keyboard Control node started.");

    // Set terminal settings to non-blocking
    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");
}

KeyboardControlNode::~KeyboardControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    char c;
    bool key_pressed = false;

    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

    // Pokud je v terminálu nějaký znak ke čtení
    if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            key_pressed = true; // Zaznamenali jsme stisk
            
            this->get_parameter("speed", current_speed_);

            if (c == '\033') { // Šipky
                char seq[2];
                if (read(STDIN_FILENO, &seq, 2) == 2 && seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A': current_twist_.linear.x = current_speed_;  break;  // Nahoru
                        case 'B': current_twist_.linear.x = -current_speed_; break;  // Dolu
                        case 'C': current_twist_.angular.z = -current_speed_; break; // Doprava
                        case 'D': current_twist_.angular.z = current_speed_;  break; // Doleva
                    }
                }
            } 
            else if (c == 'w' || c == 'W') {
                current_speed_ += 0.1;
                this->set_parameter(rclcpp::Parameter("speed", current_speed_));
                RCLCPP_INFO(this->get_logger(), "Rychlost: %.1f", current_speed_);
            } 
            else if (c == 's' || c == 'S') {
                current_speed_ -= 0.1;
                if (current_speed_ < 0.1) current_speed_ = 0.1;
                this->set_parameter(rclcpp::Parameter("speed", current_speed_));
                RCLCPP_INFO(this->get_logger(), "Rychlost: %.1f", current_speed_);
            }
            else if (c == ' ') { // Mezerník jako záchranná brzda
                current_twist_.linear.x = 0.0;
                current_twist_.angular.z = 0.0;
            }
        }
    }

    if (key_pressed) {
        no_key_timeout_ = 0; // Resetujeme počítadlo
        twist_publisher_->publish(current_twist_);
    } else {
        no_key_timeout_++;
        // Pokud jsme už 2 tiky (200 ms) nedostali znak, zastavíme robota
        if (no_key_timeout_ > 50) {
            current_twist_.linear.x = 0.0;
            current_twist_.angular.z = 0.0;
            twist_publisher_->publish(current_twist_);
        }
    }
}