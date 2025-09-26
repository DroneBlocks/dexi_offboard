#include "dexi_offboard/px4_offboard_manager.hpp"

PX4OffboardManager::PX4OffboardManager(const rclcpp::NodeOptions &options)
: Node("px4_offboard_manager", options),
  qos_profile_(1)
{
    // Configure QoS profile
    qos_profile_ = qos_profile_.best_effort()
                              .transient_local()
                              .keep_last(1);

    // Initialize publishers, subscribers, and timer
    initializePublishers();
    initializeSubscribers();
    initializeTimer();

    // Initialize offboard heartbeat message
    offboard_heartbeat_.position = true;
    offboard_heartbeat_.velocity = false;
    offboard_heartbeat_.acceleration = false;
    offboard_heartbeat_.attitude = false;
    offboard_heartbeat_.body_rate = false;
}

PX4OffboardManager::~PX4OffboardManager()
{
    stopOffboardHeartbeat();
}

void PX4OffboardManager::initializePublishers()
{
    vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_profile_);
    
    offboard_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_profile_);
    
    trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos_profile_);
}

void PX4OffboardManager::initializeSubscribers()
{
    vehicle_status_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        qos_profile_,
        std::bind(&PX4OffboardManager::vehicleStatusCallback, this, std::placeholders::_1));

    vehicle_global_pos_subscriber_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position",
        qos_profile_,
        std::bind(&PX4OffboardManager::vehicleGlobalPosCallback, this, std::placeholders::_1));

    vehicle_local_pos_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        qos_profile_,
        std::bind(&PX4OffboardManager::vehicleLocalPosCallback, this, std::placeholders::_1));

    offboard_command_subscriber_ = create_subscription<dexi_interfaces::msg::OffboardNavCommand>(
        "offboard_manager",
        qos_profile_,
        std::bind(&PX4OffboardManager::handleOffboardCommand, this, std::placeholders::_1));
}

void PX4OffboardManager::initializeTimer()
{
    const double exec_frequency = 20.0; // Hz
    const std::chrono::nanoseconds timer_period{static_cast<int64_t>(1e9/exec_frequency)};
    
    frame_timer_ = create_wall_timer(
        timer_period,
        std::bind(&PX4OffboardManager::execFrame, this));
}

void PX4OffboardManager::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    if (!prev_vehicle_status_msg_) {
        // First message received
        RCLCPP_INFO(get_logger(), "Initial vehicle status received");
    } else {
        // Check arming state changes
        if (prev_vehicle_status_msg_->arming_state != msg->arming_state) {
            if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
                RCLCPP_INFO(get_logger(), "Vehicle disarmed");
            } else if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                RCLCPP_INFO(get_logger(), "Vehicle armed");
            }
        }

        // Check navigation state changes
        if (prev_vehicle_status_msg_->nav_state != msg->nav_state) {
            switch (msg->nav_state) {
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
                    RCLCPP_INFO(get_logger(), "Vehicle in takeoff mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
                    RCLCPP_INFO(get_logger(), "Vehicle in land mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
                    RCLCPP_INFO(get_logger(), "Vehicle in hold mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
                    RCLCPP_INFO(get_logger(), "Vehicle in offboard mode");
                    break;
            }
        }
    }
    prev_vehicle_status_msg_ = msg;
}

void PX4OffboardManager::vehicleGlobalPosCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    lat_ = msg->lat;
    lon_ = msg->lon;
    alt_ = msg->alt;
}

void PX4OffboardManager::vehicleLocalPosCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    x_ = msg->x;
    y_ = msg->y;
    z_ = msg->z;
    heading_ = msg->heading;
}

void PX4OffboardManager::handleOffboardCommand(const dexi_interfaces::msg::OffboardNavCommand::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received command: %s", msg->command.c_str());
    float distance_or_degrees = msg->distance_or_degrees ? msg->distance_or_degrees : 1.0f;

    if (msg->command == "start_offboard_heartbeat") {
        startOffboardHeartbeat();
    } else if (msg->command == "stop_offboard_heartbeat") {
        stopOffboardHeartbeat();
    } else if (msg->command == "arm") {
        arm();
    } else if (msg->command == "disarm") {
        disarm();
    } else if (msg->command == "takeoff") {
        takeoff(distance_or_degrees);
    } else if (msg->command == "offboard_takeoff") {
        offboardTakeoff(distance_or_degrees);
    } else if (msg->command == "land") {
        land();
    } else if (msg->command == "fly_forward") {
        flyForward(distance_or_degrees);
    } else if (msg->command == "fly_backward") {
        flyBackward(distance_or_degrees);
    } else if (msg->command == "fly_left") {
        flyLeft(distance_or_degrees);
    } else if (msg->command == "fly_right") {
        flyRight(distance_or_degrees);
    } else if (msg->command == "fly_up") {
        flyUp(distance_or_degrees);
    } else if (msg->command == "fly_down") {
        flyDown(distance_or_degrees);
    } else if (msg->command == "yaw_left") {
        yawLeft(distance_or_degrees);
    } else if (msg->command == "yaw_right") {
        yawRight(distance_or_degrees);
    } else {
        RCLCPP_WARN(get_logger(), "Unknown command: %s", msg->command.c_str());
    }
}

void PX4OffboardManager::execFrame()
{
    // This method can be expanded to include state machine logic
    // Currently kept minimal for basic functionality
}

uint64_t PX4OffboardManager::getTimestamp()
{
    return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
}

void PX4OffboardManager::sendVehicleCommand(px4_msgs::msg::VehicleCommand& msg)
{
    msg.timestamp = getTimestamp();
    msg.source_system = 1;
    msg.source_component = 1;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.from_external = true;

    vehicle_command_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Sent vehicle command %d", msg.command);
}

void PX4OffboardManager::arm()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.param1 = 1.0f;  // 1 = arm
    msg.param2 = 0.0f;  // 0 = normal, 21196 = force
    sendVehicleCommand(msg);
}

void PX4OffboardManager::disarm()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.param1 = 0.0f;  // 0 = disarm
    msg.param2 = 0.0f;
    sendVehicleCommand(msg);
}

void PX4OffboardManager::takeoff(float altitude)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
    msg.param1 = -1.0f;
    msg.param2 = 0.0f;
    msg.param3 = 0.0f;
    msg.param4 = NAN;
    msg.param5 = NAN;
    msg.param6 = NAN;
    msg.param7 = altitude + static_cast<float>(alt_);
    sendVehicleCommand(msg);
}

void PX4OffboardManager::offboardTakeoff(float altitude)
{
    RCLCPP_INFO(get_logger(), "Starting offboard takeoff to %.2f meters", altitude);

    // Start offboard heartbeat if not already running
    if (!offboard_heartbeat_thread_run_flag_) {
        startOffboardHeartbeat();
        // Brief delay to ensure offboard mode is established
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Set target altitude setpoint (negative Z is up in NED frame)
    target_z_ = z_ - altitude;  // Current altitude minus desired climb
    target_x_ = x_;             // Hold current X position
    target_y_ = y_;             // Hold current Y position
    target_heading_ = heading_; // Hold current heading

    RCLCPP_INFO(get_logger(), "Offboard takeoff setpoint: target_z=%.2f (climb %.2f meters)",
                target_z_, altitude);
}

void PX4OffboardManager::land()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
    sendVehicleCommand(msg);
}

void PX4OffboardManager::enableOffboardMode()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1 = 1.0f;
    msg.param2 = 6.0f;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
    sendVehicleCommand(msg);
}

void PX4OffboardManager::enableHoldMode()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1 = 1.0f;
    msg.param2 = 4.0f;  // PX4_CUSTOM_MAIN_MODE_AUTO
    msg.param3 = 3.0f;  // AUTO_LOITER
    sendVehicleCommand(msg);
}

void PX4OffboardManager::sendTrajectorySetpointPosition(float x, float y, float z, float yaw)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = getTimestamp();
    msg.position = {x, y, z};
    msg.yaw = std::isnan(yaw) ? heading_ : yaw;
    trajectory_setpoint_publisher_->publish(msg);
}

void PX4OffboardManager::startOffboardHeartbeat()
{
    if (offboard_heartbeat_thread_run_flag_) {
        return;
    }

    RCLCPP_INFO(get_logger(), "Starting offboard heartbeat");

    // Initialize target setpoints to current position
    target_x_ = x_;
    target_y_ = y_;
    target_z_ = z_;
    target_heading_ = heading_;

    offboard_heartbeat_thread_run_flag_ = true;
    offboard_heartbeat_thread_ = std::make_unique<std::thread>(
        &PX4OffboardManager::sendOffboardHeartbeat, this);

    // Create timer to enable offboard mode after 1 second
    offboard_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            this->enableOffboardMode();
            // Stop the timer after first execution
            offboard_timer_->cancel();
        }
    );
}

void PX4OffboardManager::stopOffboardHeartbeat()
{
    if (!offboard_heartbeat_thread_run_flag_) {
        return;
    }

    offboard_heartbeat_thread_run_flag_ = false;
    if (offboard_heartbeat_thread_ && offboard_heartbeat_thread_->joinable()) {
        offboard_heartbeat_thread_->join();
    }
}

void PX4OffboardManager::sendOffboardHeartbeat()
{
    const std::chrono::milliseconds sleep_duration(50);  // 20Hz
    while (offboard_heartbeat_thread_run_flag_) {
        // Send offboard control mode
        offboard_heartbeat_.timestamp = getTimestamp();
        offboard_mode_publisher_->publish(offboard_heartbeat_);

        // Send target setpoints to PX4 (use current position if no target set)
        sendTrajectorySetpointPosition(target_x_, target_y_, target_z_, target_heading_);

        std::this_thread::sleep_for(sleep_duration);
    }
}

// Movement methods
void PX4OffboardManager::flyForward(float distance)
{
    target_x_ = distance * std::cos(heading_) + x_;
    target_y_ = distance * std::sin(heading_) + y_;
    target_z_ = z_;
    target_heading_ = heading_;
    RCLCPP_INFO(get_logger(), "Setting target FORWARD: %.2f meters", distance);
}

void PX4OffboardManager::flyBackward(float distance)
{
    target_x_ = distance * std::cos(heading_ + M_PI) + x_;
    target_y_ = distance * std::sin(heading_ + M_PI) + y_;
    target_z_ = z_;
    target_heading_ = heading_;
    RCLCPP_INFO(get_logger(), "Setting target BACKWARD: %.2f meters", distance);
}

void PX4OffboardManager::flyRight(float distance)
{
    target_x_ = distance * std::cos(heading_ + M_PI_2) + x_;
    target_y_ = distance * std::sin(heading_ + M_PI_2) + y_;
    target_z_ = z_;
    target_heading_ = heading_;
    RCLCPP_INFO(get_logger(), "Setting target RIGHT: %.2f meters", distance);
}

void PX4OffboardManager::flyLeft(float distance)
{
    target_x_ = distance * std::cos(heading_ - M_PI_2) + x_;
    target_y_ = distance * std::sin(heading_ - M_PI_2) + y_;
    target_z_ = z_;
    target_heading_ = heading_;
    RCLCPP_INFO(get_logger(), "Setting target LEFT: %.2f meters", distance);
}

void PX4OffboardManager::flyUp(float distance)
{
    target_z_ = z_ - distance;  // Negative Z is up in NED
    target_x_ = x_;
    target_y_ = y_;
    target_heading_ = heading_;
    RCLCPP_INFO(get_logger(), "Setting altitude target: %.2f meters", -target_z_);
}

void PX4OffboardManager::flyDown(float distance)
{
    target_z_ = z_ + distance;  // Positive Z is down in NED
    target_x_ = x_;
    target_y_ = y_;
    target_heading_ = heading_;
    RCLCPP_INFO(get_logger(), "Setting altitude target: %.2f meters", -target_z_);
}

void PX4OffboardManager::yawLeft(float angle)
{
    target_heading_ = heading_ - angle * M_PI / 180.0f;
    target_x_ = x_;
    target_y_ = y_;
    target_z_ = z_;
    RCLCPP_INFO(get_logger(), "Setting yaw target: %.2f degrees", target_heading_ * 180.0f / M_PI);
}

void PX4OffboardManager::yawRight(float angle)
{
    target_heading_ = heading_ + angle * M_PI / 180.0f;
    target_x_ = x_;
    target_y_ = y_;
    target_z_ = z_;
    RCLCPP_INFO(get_logger(), "Setting yaw target: %.2f degrees", target_heading_ * 180.0f / M_PI);
}

// Main function
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4OffboardManager>());
    rclcpp::shutdown();
    return 0;
}
