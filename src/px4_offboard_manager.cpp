#include "dexi_offboard/px4_offboard_manager.hpp"

PX4OffboardManager::PX4OffboardManager(const rclcpp::NodeOptions &options)
: Node("px4_offboard_manager", options),
  qos_profile_(1)
{
    // Declare parameters
    this->declare_parameter("keyboard_control_enabled", false);

    // Get parameter value
    keyboard_control_enabled_ = this->get_parameter("keyboard_control_enabled").as_bool();
    RCLCPP_INFO(get_logger(), "Keyboard control enabled: %s", keyboard_control_enabled_ ? "true" : "false");

    // Configure QoS profile
    qos_profile_ = qos_profile_.best_effort()
                              .transient_local()
                              .keep_last(1);

    // Create callback group for service to run in separate thread
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Initialize publishers, subscribers, services, and timer
    initializePublishers();
    initializeSubscribers();
    initializeServices();
    initializeTimer();

    // Initialize mission controller
    mission_controller_ = std::make_unique<MissionController>();

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

void PX4OffboardManager::initializeServices()
{
    blockly_command_service_ = create_service<dexi_interfaces::srv::ExecuteBlocklyCommand>(
        "execute_blockly_command",
        std::bind(&PX4OffboardManager::executeBlocklyCommandCallback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_);
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

    // If no active target and keyboard control is enabled, update target to hold current position
    // This prevents drift when using keyboard control
    if (!target_active_ && keyboard_control_enabled_ && offboard_heartbeat_thread_run_flag_) {
        target_x_ = x_;
        target_y_ = y_;
        target_z_ = z_;
        target_heading_ = heading_;
    }

    // Check if target is reached after position update
    if (target_active_ && isTargetReached()) {
        RCLCPP_INFO(get_logger(), "Target reached! Position: (%.2f, %.2f, %.2f), Heading: %.2f°",
                   x_, y_, z_, heading_ * 180.0 / M_PI);
        clearTarget();
    }
}

bool PX4OffboardManager::isTargetReached()
{
    if (!target_active_) return false;

    // Calculate position distance
    double dx = x_ - target_x_;
    double dy = y_ - target_y_;
    double dz = z_ - target_z_;
    double position_error = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Calculate heading error (handle wraparound)
    double heading_error = std::abs(heading_ - target_heading_);
    if (heading_error > M_PI) {
        heading_error = 2.0 * M_PI - heading_error;
    }

    return (position_error <= position_tolerance_) && (heading_error <= heading_tolerance_);
}

void PX4OffboardManager::setTarget(double x, double y, double z, double heading)
{
    target_x_ = x;
    target_y_ = y;
    target_z_ = z;
    target_heading_ = heading;
    target_active_ = true;
}

void PX4OffboardManager::clearTarget()
{
    target_active_ = false;

    // Check if we have an active mission
    if (mission_controller_->isMissionActive()) {
        mission_controller_->waypointReached();

        // Add delay before next waypoint for drone settling
        if (mission_controller_->hasNextWaypoint()) {
            RCLCPP_INFO(get_logger(), "Waypoint reached! Settling for %.1f seconds before next waypoint...",
                       mission_waypoint_delay_);

            mission_delay_timer_ = create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(mission_waypoint_delay_ * 1000)),
                [this]() {
                    executeMission();  // Continue to next waypoint after delay
                    mission_delay_timer_->cancel();  // Stop the timer
                }
            );
        } else {
            // No more waypoints, mission complete
            RCLCPP_INFO(get_logger(), "Mission completed!");
        }
    }
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
    } else if (msg->command == "box_mission") {
        startBoxMission(distance_or_degrees);
    } else if (msg->command == "stop_mission") {
        stopMission();
    } else if (msg->command == "set_mission_delay") {
        mission_waypoint_delay_ = distance_or_degrees;
        RCLCPP_INFO(get_logger(), "Mission waypoint delay set to %.1f seconds", mission_waypoint_delay_);
    } else {
        RCLCPP_WARN(get_logger(), "Unknown command: %s", msg->command.c_str());
    }
}

void PX4OffboardManager::executeBlocklyCommandCallback(
    const std::shared_ptr<dexi_interfaces::srv::ExecuteBlocklyCommand::Request> request,
    std::shared_ptr<dexi_interfaces::srv::ExecuteBlocklyCommand::Response> response)
{
    auto start_time = std::chrono::steady_clock::now();


    // Execute the command based on the command string
    bool command_has_target = true;  // Most commands have targets

    if (request->command == "arm") {
        arm();
        command_has_target = false;  // Immediate command
    } else if (request->command == "disarm") {
        disarm();
        command_has_target = false;  // Immediate command
    } else if (request->command == "start_offboard_heartbeat") {
        startOffboardHeartbeat();
        command_has_target = false;  // Immediate command
    } else if (request->command == "stop_offboard_heartbeat") {
        stopOffboardHeartbeat();
        command_has_target = false;  // Immediate command
    } else if (request->command == "takeoff") {
        takeoff(request->parameter);
        command_has_target = false;  // Auto takeoff, not tracked by target_active_
    } else if (request->command == "offboard_takeoff") {
        offboardTakeoff(request->parameter);
        command_has_target = true;  // Offboard takeoff uses target tracking
    } else if (request->command == "land") {
        land();
        command_has_target = false;  // Auto land, not tracked
    } else if (request->command == "fly_forward") {
        flyForward(request->parameter);
    } else if (request->command == "fly_backward") {
        flyBackward(request->parameter);
    } else if (request->command == "fly_left") {
        flyLeft(request->parameter);
    } else if (request->command == "fly_right") {
        flyRight(request->parameter);
    } else if (request->command == "fly_up") {
        flyUp(request->parameter);
    } else if (request->command == "fly_down") {
        flyDown(request->parameter);
    } else if (request->command == "yaw_left") {
        yawLeft(request->parameter);
    } else if (request->command == "yaw_right") {
        yawRight(request->parameter);
    } else {
        RCLCPP_WARN(get_logger(), "Unknown Blockly command: %s", request->command.c_str());
        response->success = false;
        response->message = "Unknown command: " + request->command;
        response->execution_time = 0.0;
        return;
    }

    // If the command doesn't have a target (immediate commands), return immediately
    if (!command_has_target) {
        auto end_time = std::chrono::steady_clock::now();
        response->success = true;
        response->message = "Command completed";
        response->execution_time = std::chrono::duration<float>(end_time - start_time).count();
        return;
    }

    // Wait for target to be reached
    const std::chrono::milliseconds poll_interval(50);
    auto timeout_duration = std::chrono::duration<float>(request->timeout);
    bool timeout_enabled = request->timeout > 0.0f;

    while (rclcpp::ok() && target_active_) {
        // Check for timeout
        if (timeout_enabled) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed >= timeout_duration) {
                clearTarget();
                response->success = false;
                response->message = "Command timed out";
                response->execution_time = std::chrono::duration<float>(elapsed).count();
                return;
            }
        }

        std::this_thread::sleep_for(poll_interval);
    }

    // Command completed successfully
    auto end_time = std::chrono::steady_clock::now();
    response->success = true;
    response->message = "Target reached";
    response->execution_time = std::chrono::duration<float>(end_time - start_time).count();

    // Add settling delay after reaching target
    std::this_thread::sleep_for(std::chrono::seconds(1));
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

    // Wait for valid heading data (up to 2 seconds)
    // Heading of exactly 0.0 likely means we haven't received position data yet
    auto start_time = std::chrono::steady_clock::now();
    while (heading_ == 0.0 && x_ == 0.0 && y_ == 0.0 && z_ == 0.0) {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > std::chrono::seconds(2)) {
            RCLCPP_WARN(get_logger(), "No valid position/heading received after 2s, using current values");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Set target altitude setpoint (negative Z is up in NED frame)
    target_z_ = z_ - altitude;  // Current altitude minus desired climb
    target_x_ = x_;             // Hold current X position
    target_y_ = y_;             // Hold current Y position
    target_heading_ = heading_; // Hold current heading
    target_active_ = true;      // Mark target as active

    RCLCPP_INFO(get_logger(), "Offboard takeoff setpoint: target_z=%.2f (climb %.2f meters), heading=%.2f rad",
                target_z_, altitude, target_heading_);
}

void PX4OffboardManager::land()
{
    // Stop any active mission first
    if (mission_controller_->isMissionActive()) {
        stopMission();
    }

    // Stop offboard heartbeat since we're switching to auto land mode
    stopOffboardHeartbeat();

    // Send land command to PX4
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
    sendVehicleCommand(msg);

    RCLCPP_INFO(get_logger(), "Landing initiated - offboard heartbeat stopped");
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
    double new_x = distance * std::cos(heading_) + x_;
    double new_y = distance * std::sin(heading_) + y_;
    setTarget(new_x, new_y, z_, heading_);
    RCLCPP_INFO(get_logger(), "Flying FORWARD: %.2f meters", distance);
}

void PX4OffboardManager::flyBackward(float distance)
{
    double new_x = distance * std::cos(heading_ + M_PI) + x_;
    double new_y = distance * std::sin(heading_ + M_PI) + y_;
    setTarget(new_x, new_y, z_, heading_);
    RCLCPP_INFO(get_logger(), "Flying BACKWARD: %.2f meters", distance);
}

void PX4OffboardManager::flyRight(float distance)
{
    double new_x = distance * std::cos(heading_ + M_PI_2) + x_;
    double new_y = distance * std::sin(heading_ + M_PI_2) + y_;
    setTarget(new_x, new_y, z_, heading_);
    RCLCPP_INFO(get_logger(), "Flying RIGHT: %.2f meters", distance);
}

void PX4OffboardManager::flyLeft(float distance)
{
    double new_x = distance * std::cos(heading_ - M_PI_2) + x_;
    double new_y = distance * std::sin(heading_ - M_PI_2) + y_;
    setTarget(new_x, new_y, z_, heading_);
    RCLCPP_INFO(get_logger(), "Flying LEFT: %.2f meters", distance);
}

void PX4OffboardManager::flyUp(float distance)
{
    double new_z = z_ - distance;  // Negative Z is up in NED
    setTarget(x_, y_, new_z, heading_);
    RCLCPP_INFO(get_logger(), "Flying UP: %.2f meters", distance);
}

void PX4OffboardManager::flyDown(float distance)
{
    double new_z = z_ + distance;  // Positive Z is down in NED
    setTarget(x_, y_, new_z, heading_);
    RCLCPP_INFO(get_logger(), "Flying DOWN: %.2f meters", distance);
}

void PX4OffboardManager::yawLeft(float angle)
{
    double new_heading = heading_ - angle * M_PI / 180.0f;
    setTarget(x_, y_, z_, new_heading);
    RCLCPP_INFO(get_logger(), "Yawing LEFT: %.2f degrees", angle);
}

void PX4OffboardManager::yawRight(float angle)
{
    double new_heading = heading_ + angle * M_PI / 180.0f;
    setTarget(x_, y_, z_, new_heading);
    RCLCPP_INFO(get_logger(), "Yawing RIGHT: %.2f degrees", angle);
}

// Mission control methods
void PX4OffboardManager::startBoxMission(float size)
{
    mission_controller_->createBoxMission(size, x_, y_, z_);

    auto target_callback = [this]() {
        RCLCPP_INFO(get_logger(), "Waypoint reached! Continuing mission...");
    };

    auto complete_callback = [this]() {
        RCLCPP_INFO(get_logger(), "Mission completed!");
    };

    mission_controller_->startMission(target_callback, complete_callback);

    RCLCPP_INFO(get_logger(), "Starting box mission: %.2fm x %.2fm at current altitude %.2fm",
               size, size, -z_);  // Display positive altitude (negative Z in NED)

    executeMission();
}

void PX4OffboardManager::stopMission()
{
    mission_controller_->stopMission();
    clearTarget();
    RCLCPP_INFO(get_logger(), "Mission stopped");
}

void PX4OffboardManager::executeMission()
{
    if (!mission_controller_->isMissionActive() || !mission_controller_->hasNextWaypoint()) {
        return;
    }

    Waypoint next = mission_controller_->getNextWaypoint();
    setTarget(next.x, next.y, next.z, next.heading);

    RCLCPP_INFO(get_logger(), "Executing waypoint: %s -> (%.2f, %.2f, %.2f)",
               next.description.c_str(), next.x, next.y, next.z);
}

// Main function
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Use MultiThreadedExecutor to allow service callbacks and subscribers
    // to run concurrently - this is essential for the blocking service call
    // to work while position updates continue to be processed
    auto node = std::make_shared<PX4OffboardManager>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
