#ifndef DEXI_OFFBOARD__PX4_OFFBOARD_MANAGER_HPP_
#define DEXI_OFFBOARD__PX4_OFFBOARD_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <dexi_interfaces/msg/offboard_nav_command.hpp>
#include <dexi_interfaces/srv/execute_blockly_command.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <cmath>
#include <chrono>

class PX4OffboardManager : public rclcpp::Node
{
public:
    PX4OffboardManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~PX4OffboardManager();

private:
    // QoS profile
    rclcpp::QoS qos_profile_;

    // ROS publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    // ROS subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_pos_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_subscriber_;
    rclcpp::Subscription<dexi_interfaces::msg::OffboardNavCommand>::SharedPtr offboard_command_subscriber_;

    // ROS services
    rclcpp::Service<dexi_interfaces::srv::ExecuteBlocklyCommand>::SharedPtr blockly_command_service_;

    // Callback groups for concurrent execution
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    // Timer
    rclcpp::TimerBase::SharedPtr frame_timer_;
    rclcpp::TimerBase::SharedPtr offboard_timer_; 

    // State variables
    std::shared_ptr<px4_msgs::msg::VehicleStatus> prev_vehicle_status_msg_;
    double lat_{0.0}, lon_{0.0}, alt_{0.0};
    double x_{0.0}, y_{0.0}, z_{0.0}, heading_{0.0};

    // Target setpoints for offboard control
    double target_x_{0.0}, target_y_{0.0}, target_z_{0.0}, target_heading_{0.0};

    // Target reached detection
    bool target_active_{false};
    double position_tolerance_{0.25};  // meters
    double heading_tolerance_{0.1};   // radians (~5.7 degrees)

    // Parameters
    bool keyboard_control_enabled_{false};

    // Offboard control
    std::atomic<bool> offboard_heartbeat_thread_run_flag_{false};
    std::unique_ptr<std::thread> offboard_heartbeat_thread_;
    px4_msgs::msg::OffboardControlMode offboard_heartbeat_;

    // Methods
    void initializePublishers();
    void initializeSubscribers();
    void initializeServices();
    void initializeTimer();

    // Callbacks
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicleGlobalPosCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void vehicleLocalPosCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void handleOffboardCommand(const dexi_interfaces::msg::OffboardNavCommand::SharedPtr msg);
    void executeBlocklyCommandCallback(
        const std::shared_ptr<dexi_interfaces::srv::ExecuteBlocklyCommand::Request> request,
        std::shared_ptr<dexi_interfaces::srv::ExecuteBlocklyCommand::Response> response);
    void execFrame();

    // Command methods
    void arm();
    void disarm();
    void takeoff(float altitude);
    void offboardTakeoff(float altitude);
    void land();
    void enableOffboardMode();
    void enableHoldMode();
    void enableStabilizedMode();

    // Movement methods
    void flyForward(float distance);
    void flyBackward(float distance);
    void flyRight(float distance);
    void flyLeft(float distance);
    void flyUp(float distance);
    void flyDown(float distance);
    void yawLeft(float angle);
    void yawRight(float angle);

    // Target reached detection
    bool isTargetReached();
    void setTarget(double x, double y, double z, double heading);
    void clearTarget();

    // Trajectory-based flight
    void flyCircle(float radius);

    // Utility methods
    uint64_t getTimestamp();
    void sendVehicleCommand(px4_msgs::msg::VehicleCommand& msg);
    void sendTrajectorySetpointPosition(float x, float y, float z, float yaw = NAN);
    void startOffboardHeartbeat();
    void stopOffboardHeartbeat();
    void sendOffboardHeartbeat();
};

#endif // DEXI_OFFBOARD__PX4_OFFBOARD_MANAGER_HPP_ 