#ifndef DEXI_OFFBOARD__KEYBOARD_TELEOP_HPP_
#define DEXI_OFFBOARD__KEYBOARD_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <dexi_interfaces/msg/offboard_nav_command.hpp>
#include <termios.h>
#include <chrono>

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop();
    ~KeyboardTeleop();

private:
    void setupKeyboard();
    void restoreKeyboard();
    void timerCallback();
    void handleKeypress(char key);
    void handleArrowKey(char arrow_code);
    void printInstructions();

    // ROS2 components
    rclcpp::Publisher<dexi_interfaces::msg::OffboardNavCommand>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Terminal handling
    struct termios original_termios_;

    // Control parameters
    double yaw_increment_;
};

#endif // DEXI_OFFBOARD__KEYBOARD_TELEOP_HPP_