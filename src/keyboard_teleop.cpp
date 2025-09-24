#include "dexi_offboard/keyboard_teleop.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

KeyboardTeleop::KeyboardTeleop() : Node("keyboard_teleop")
{
    // QoS profile to match offboard manager
    auto qos_profile = rclcpp::QoS(1).best_effort().transient_local().keep_last(1);

    // Create publisher for offboard commands
    cmd_publisher_ = create_publisher<dexi_interfaces::msg::OffboardNavCommand>(
        "/dexi/offboard_manager", qos_profile);

    // Parameters
    yaw_increment_ = 15.0;  // degrees per keypress

    // Setup non-blocking keyboard input
    setupKeyboard();

    // Timer for checking keyboard input
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),  // 20Hz
        std::bind(&KeyboardTeleop::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Keyboard Teleop initialized");
    printInstructions();
}

KeyboardTeleop::~KeyboardTeleop()
{
    restoreKeyboard();
}

void KeyboardTeleop::setupKeyboard()
{
    // Save original terminal settings
    tcgetattr(STDIN_FILENO, &original_termios_);

    // Set terminal to raw mode for immediate key detection
    struct termios raw = original_termios_;
    raw.c_lflag &= ~(ECHO | ICANON);  // Disable echo and canonical mode
    raw.c_cc[VMIN] = 0;   // Non-blocking read
    raw.c_cc[VTIME] = 0;  // No timeout

    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void KeyboardTeleop::restoreKeyboard()
{
    // Restore original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
}

void KeyboardTeleop::timerCallback()
{
    char key;
    if (read(STDIN_FILENO, &key, 1) > 0) {
        // Check for escape sequences (arrow keys)
        if (key == '\033') {
            char seq[2];
            if (read(STDIN_FILENO, &seq[0], 1) > 0 && seq[0] == '[') {
                if (read(STDIN_FILENO, &seq[1], 1) > 0) {
                    handleArrowKey(seq[1]);
                    return;
                }
            }
        }
        handleKeypress(key);
    }
}

void KeyboardTeleop::handleKeypress(char key)
{
    auto msg = dexi_interfaces::msg::OffboardNavCommand();

    switch (key) {
        case 'w':
        case 'W':
            msg.command = "fly_up";
            msg.distance_or_degrees = 1.0f;  // 1 meter up
            cmd_publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "Moving UP 1.0 meter");
            break;

        case 's':
        case 'S':
            msg.command = "fly_down";
            msg.distance_or_degrees = 1.0f;  // 1 meter down
            cmd_publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "Moving DOWN 1.0 meter");
            break;

        case 'a':
        case 'A':
            msg.command = "yaw_left";
            msg.distance_or_degrees = yaw_increment_;
            cmd_publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "Yaw LEFT %.1f degrees", yaw_increment_);
            break;

        case 'd':
        case 'D':
            msg.command = "yaw_right";
            msg.distance_or_degrees = yaw_increment_;
            cmd_publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "Yaw RIGHT %.1f degrees", yaw_increment_);
            break;

        case 'h':
        case 'H':
            printInstructions();
            break;

        case 'q':
        case 'Q':
        case '\x03':  // Ctrl+C
            RCLCPP_INFO(get_logger(), "Exiting keyboard teleop...");
            rclcpp::shutdown();
            break;

        default:
            // Ignore other keys
            break;
    }
}

void KeyboardTeleop::handleArrowKey(char arrow_code)
{
    auto msg = dexi_interfaces::msg::OffboardNavCommand();
    msg.distance_or_degrees = 1.0f;  // 1 meter for all movements

    switch (arrow_code) {
        case 'A':  // Up arrow - Pitch forward
            msg.command = "fly_forward";
            cmd_publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "PITCH FORWARD 1.0 meter (↑)");
            break;

        case 'B':  // Down arrow - Pitch backward
            msg.command = "fly_backward";
            cmd_publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "PITCH BACKWARD 1.0 meter (↓)");
            break;

        case 'C':  // Right arrow - Roll right
            msg.command = "fly_right";
            cmd_publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "ROLL RIGHT 1.0 meter (→)");
            break;

        case 'D':  // Left arrow - Roll left
            msg.command = "fly_left";
            cmd_publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "ROLL LEFT 1.0 meter (←)");
            break;

        default:
            // Ignore other escape sequences
            break;
    }
}

void KeyboardTeleop::printInstructions()
{
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "       DEXI Keyboard Teleop - Transmitter Style" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << std::endl;
    std::cout << "LEFT STICK (Throttle/Yaw):" << std::endl;
    std::cout << "  W / w  : THROTTLE UP   (Move UP 1.0m)" << std::endl;
    std::cout << "  S / s  : THROTTLE DOWN (Move DOWN 1.0m)" << std::endl;
    std::cout << "  A / a  : YAW LEFT      (" << yaw_increment_ << "°)" << std::endl;
    std::cout << "  D / d  : YAW RIGHT     (" << yaw_increment_ << "°)" << std::endl;
    std::cout << std::endl;
    std::cout << "RIGHT STICK (Pitch/Roll): [Arrow Keys]" << std::endl;
    std::cout << "  ↑     : PITCH FORWARD  (Move FORWARD 1.0m)" << std::endl;
    std::cout << "  ↓     : PITCH BACKWARD (Move BACKWARD 1.0m)" << std::endl;
    std::cout << "  ←     : ROLL LEFT      (Move LEFT 1.0m)" << std::endl;
    std::cout << "  →     : ROLL RIGHT     (Move RIGHT 1.0m)" << std::endl;
    std::cout << std::endl;
    std::cout << "System Controls:" << std::endl;
    std::cout << "  H / h  : Show this help" << std::endl;
    std::cout << "  Q / q  : Quit teleop" << std::endl;
    std::cout << std::endl;
    std::cout << "NOTE: Make sure offboard heartbeat is started!" << std::endl;
    std::cout << "      ros2 topic pub --once /dexi/offboard_manager \\" << std::endl;
    std::cout << "        dexi_interfaces/msg/OffboardNavCommand \\" << std::endl;
    std::cout << "        \"{command: 'start_offboard_heartbeat'}\"" << std::endl;
    std::cout << std::string(50, '=') << std::endl;
    std::cout << "Ready for input (no need to press Enter)..." << std::endl;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KeyboardTeleop>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}