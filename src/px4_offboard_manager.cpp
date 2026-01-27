#include "dexi_offboard/px4_offboard_manager.hpp"

PX4OffboardManager::PX4OffboardManager(const rclcpp::NodeOptions &options)
: Node("px4_offboard_manager", options),
  qos_profile_(1)
{
    // Declare parameters
    // Note: keyboard_control_enabled is read by the GUI to show/hide keyboard control option
    this->declare_parameter("keyboard_control_enabled", false);
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

    // Pause setpoints subscriber - allows other nodes (e.g., precision_landing) to take over control
    pause_setpoints_subscriber_ = create_subscription<std_msgs::msg::Bool>(
        "/dexi/pause_setpoints",
        10,
        std::bind(&PX4OffboardManager::handlePauseSetpoints, this, std::placeholders::_1));

    // Vehicle land detected subscriber - for proper landing confirmation
    vehicle_land_detected_subscriber_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected",
        qos_profile_,
        std::bind(&PX4OffboardManager::vehicleLandDetectedCallback, this, std::placeholders::_1));
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

    // Position hold is now set once in clearTarget() rather than continuously here.
    // Continuously updating target to current position caused altitude drift on real hardware
    // as sensor noise/drift would be amplified into sustained movement.

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
    // Set hold position ONCE at current location to prevent drift
    // This fixes altitude drift caused by continuously updating target to current position
    target_x_ = x_;
    target_y_ = y_;
    target_z_ = z_;
    target_heading_ = heading_;
    target_active_ = false;
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
    } else if (msg->command == "set_goto_ned_params") {
        setGotoNEDParams(msg->north, msg->east, msg->down, msg->yaw);
        RCLCPP_INFO(get_logger(), "Set goto_ned params: N=%.2f, E=%.2f, D=%.2f, Yaw=%.2f°",
                    msg->north, msg->east, msg->down, msg->yaw);
    } else if (msg->command == "goto_ned") {
        gotoNED(msg->north, msg->east, msg->down, msg->yaw);
    } else if (msg->command == "circle") {
        flyCircle(distance_or_degrees);
    } else if (msg->command == "switch_offboard_mode") {
        enableOffboardMode();
    } else if (msg->command == "switch_hold_mode") {
        enableHoldMode();
    } else {
        RCLCPP_WARN(get_logger(), "Unknown command: %s", msg->command.c_str());
    }
}

void PX4OffboardManager::handlePauseSetpoints(const std_msgs::msg::Bool::SharedPtr msg)
{
    setpoints_paused_ = msg->data;
    if (msg->data) {
        RCLCPP_INFO(get_logger(), "Setpoints PAUSED - external node has taken control");
    } else {
        RCLCPP_INFO(get_logger(), "Setpoints RESUMED - offboard manager has control");
    }
}

void PX4OffboardManager::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    landed_ = msg->landed;
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
        float target_altitude = request->parameter;
        takeoff(target_altitude);

        // Wait for takeoff to reach target altitude
        const std::chrono::milliseconds poll_interval(100);
        auto timeout_duration = std::chrono::duration<float>(request->timeout);
        bool timeout_enabled = request->timeout > 0.0f;

        // Initial altitude (negative Z is up in NED)
        double initial_z = z_;
        double target_z = initial_z - target_altitude;

        RCLCPP_INFO(get_logger(), "Waiting for takeoff: current_z=%.2f, target_z=%.2f", z_, target_z);

        while (rclcpp::ok()) {
            // Check if we've reached target altitude (within tolerance)
            double altitude_error = std::abs(z_ - target_z);
            if (altitude_error < position_tolerance_) {
                RCLCPP_INFO(get_logger(), "Takeoff altitude reached: z=%.2f", z_);
                break;
            }

            // Check for timeout
            if (timeout_enabled) {
                auto elapsed = std::chrono::steady_clock::now() - start_time;
                if (elapsed >= timeout_duration) {
                    response->success = false;
                    response->message = "Takeoff timed out - altitude not reached";
                    response->execution_time = std::chrono::duration<float>(elapsed).count();
                    return;
                }
            }

            std::this_thread::sleep_for(poll_interval);
        }

        command_has_target = false;  // Already handled waiting above
    } else if (request->command == "offboard_takeoff") {
        offboardTakeoff(request->parameter);
        command_has_target = true;  // Offboard takeoff uses target tracking
    } else if (request->command == "land") {
        land();

        // Wait for landing to complete
        const std::chrono::milliseconds poll_interval(100);
        auto timeout_duration = std::chrono::duration<float>(request->timeout);
        bool timeout_enabled = request->timeout > 0.0f;

        RCLCPP_INFO(get_logger(), "Waiting for landing to complete...");

        while (rclcpp::ok()) {
            // Check both PX4's land detector AND altitude as fallback
            // This ensures landing detection works in both real hardware and simulation
            bool altitude_landed = std::abs(z_) < 0.5;

            if (landed_ || altitude_landed) {
                if (landed_) {
                    RCLCPP_INFO(get_logger(), "Landing confirmed by PX4 land detector at z=%.2f", z_);
                } else {
                    RCLCPP_INFO(get_logger(), "Landing confirmed by altitude check at z=%.2f", z_);
                }

                // Brief delay to ensure stable on ground
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                // Reset home position to recalibrate altitude reference
                // This prevents altitude drift in SITL after landing
                resetHomePosition();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                break;
            }

            // Check for timeout
            if (timeout_enabled) {
                auto elapsed = std::chrono::steady_clock::now() - start_time;
                if (elapsed >= timeout_duration) {
                    response->success = false;
                    response->message = "Landing timed out";
                    response->execution_time = std::chrono::duration<float>(elapsed).count();
                    return;
                }
            }

            std::this_thread::sleep_for(poll_interval);
        }

        command_has_target = false;  // Already handled waiting above
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
    } else if (request->command == "goto_ned") {
        // Use pending values that should have been set via setGotoNEDParams
        gotoNED(pending_north_, pending_east_, pending_down_, pending_yaw_);
    } else if (request->command == "circle") {
        // flyCircle is a blocking call that runs the entire trajectory
        flyCircle(request->parameter);
        command_has_target = false;  // Already completed
    } else if (request->command == "switch_offboard_mode") {
        enableOffboardMode();
        command_has_target = false;  // Immediate command
    } else if (request->command == "switch_hold_mode") {
        enableHoldMode();
        command_has_target = false;  // Immediate command
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
    // Start offboard heartbeat signal if not already running
    // This is needed for PX4 SITL to accept arm commands without RC
    // Note: This only starts the heartbeat signal, NOT offboard flight mode
    // The "switch to offboard mode" block will enable actual offboard flight mode
    if (!offboard_heartbeat_thread_run_flag_) {
        RCLCPP_INFO(get_logger(), "Starting offboard signal for arming...");

        // Initialize target setpoints to current position (hold position)
        target_x_ = x_;
        target_y_ = y_;
        target_z_ = z_;
        target_heading_ = heading_;

        // Start the heartbeat thread
        offboard_heartbeat_thread_run_flag_ = true;
        offboard_heartbeat_thread_ = std::make_unique<std::thread>(
            &PX4OffboardManager::sendOffboardHeartbeat, this);

        // Wait for PX4 to recognize the offboard signal
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.param1 = 1.0f;  // 1 = arm
    msg.param2 = 0.0f;  // Normal arm - offboard heartbeat provides control signal
    sendVehicleCommand(msg);
    RCLCPP_INFO(get_logger(), "Arm command sent");
}

void PX4OffboardManager::disarm()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.param1 = 0.0f;  // 0 = disarm
    msg.param2 = 0.0f;  // Normal disarm (force disarm causes 60s re-arm lockout)
    sendVehicleCommand(msg);
    RCLCPP_INFO(get_logger(), "Disarm command sent");
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

    // Ensure offboard mode is enabled
    // This handles the case where arm() started heartbeat but didn't enable offboard mode
    if (!offboard_heartbeat_thread_run_flag_) {
        startOffboardHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
        // Heartbeat running (from arm), just enable offboard mode
        enableOffboardMode();
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
    // Stop offboard heartbeat since we're switching to auto land mode
    stopOffboardHeartbeat();

    // Send land command to PX4
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
    sendVehicleCommand(msg);

    RCLCPP_INFO(get_logger(), "Landing initiated - offboard heartbeat stopped");
}

void PX4OffboardManager::resetHomePosition()
{
    // Reset home position to current location
    // This resets the local frame origin, which helps prevent altitude drift after landing
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME;
    msg.param1 = 1.0f;  // 1 = use current location
    sendVehicleCommand(msg);
    RCLCPP_INFO(get_logger(), "Home position reset to current location");
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
    // If heartbeat already running (e.g., started by arm()), just enable offboard mode
    if (offboard_heartbeat_thread_run_flag_) {
        RCLCPP_INFO(get_logger(), "Heartbeat already running, enabling offboard mode");
        enableOffboardMode();
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
        // Always send offboard control mode (keeps offboard mode active)
        offboard_heartbeat_.timestamp = getTimestamp();
        offboard_mode_publisher_->publish(offboard_heartbeat_);

        // Only send setpoints if not paused (allows external nodes to control)
        if (!setpoints_paused_) {
            sendTrajectorySetpointPosition(target_x_, target_y_, target_z_, target_heading_);
        }

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

void PX4OffboardManager::gotoNED(float north, float east, float down, float yaw)
{
    double yaw_rad = yaw * M_PI / 180.0f;
    setTarget(north, east, down, yaw_rad);
    RCLCPP_INFO(get_logger(), "Going to NED position: N=%.2f, E=%.2f, D=%.2f, Yaw=%.2f°",
                north, east, down, yaw);
}

void PX4OffboardManager::setGotoNEDParams(float north, float east, float down, float yaw)
{
    pending_north_ = north;
    pending_east_ = east;
    pending_down_ = down;
    pending_yaw_ = yaw;
}

// Trajectory-based flight methods
void PX4OffboardManager::flyCircle(float radius)
{
    RCLCPP_INFO(get_logger(), "Starting smooth circle flight: radius %.2fm at altitude %.2fm",
               radius, -z_);

    // Remember if heartbeat was running so we can restore it later
    bool heartbeat_was_running = offboard_heartbeat_thread_run_flag_;

    // Stop the background heartbeat - we'll take over sending setpoints at 20Hz
    if (heartbeat_was_running) {
        RCLCPP_INFO(get_logger(), "Pausing offboard heartbeat for trajectory control");
        stopOffboardHeartbeat();
    }

    // Store starting position and heading
    const double center_x = x_;
    const double center_y = y_;
    const double center_z = z_;
    const double start_heading = heading_;

    // Circle parameters - scale duration with radius to maintain reasonable speed
    // Target speed: ~1.0 m/s for smooth flight
    const double circumference = 2.0 * M_PI * radius;
    const double target_speed = 1.0;  // m/s - conservative for good tracking
    const double duration = circumference / target_speed;  // Scale time with radius
    const double angular_velocity = 2.0 * M_PI / duration;  // radians per second
    const int rate_hz = 20;  // 20Hz control rate
    const std::chrono::milliseconds dt(1000 / rate_hz);

    // Calculate total number of steps
    const int total_steps = static_cast<int>(duration * rate_hz);

    RCLCPP_INFO(get_logger(),
               "Circle: radius=%.1fm, circumference=%.1fm, duration=%.1fs, speed=%.2fm/s, setpoints=%d",
               radius, circumference, duration, target_speed, total_steps);

    // Fly the circle
    for (int step = 0; step <= total_steps && rclcpp::ok(); ++step) {
        double t = step * (1.0 / rate_hz);  // Current time in seconds
        double angle = angular_velocity * t;  // Current angle around circle

        // Calculate position in body frame (relative to starting heading)
        // Circle starts going forward (positive X in body frame)
        double x_body = radius * std::sin(angle);
        double y_body = radius * (1.0 - std::cos(angle));

        // Rotate by starting heading to get NED frame coordinates
        double cos_heading = std::cos(start_heading);
        double sin_heading = std::sin(start_heading);

        double x_ned = center_x + (x_body * cos_heading - y_body * sin_heading);
        double y_ned = center_y + (x_body * sin_heading + y_body * cos_heading);

        // Calculate heading tangent to circle (facing forward along path)
        double target_heading = start_heading + angle;

        // Send offboard control mode to keep offboard mode active
        offboard_heartbeat_.timestamp = getTimestamp();
        offboard_mode_publisher_->publish(offboard_heartbeat_);

        // Send trajectory setpoint for this point on the circle
        sendTrajectorySetpointPosition(
            static_cast<float>(x_ned),
            static_cast<float>(y_ned),
            static_cast<float>(center_z),
            static_cast<float>(target_heading)
        );

        // Log progress and tracking error every 2 seconds
        if (step % (rate_hz * 2) == 0) {
            double progress = (angle / (2.0 * M_PI)) * 100.0;

            // Calculate tracking error (how far drone is from commanded position)
            double pos_error_x = x_ - x_ned;
            double pos_error_y = y_ - y_ned;
            double pos_error_z = z_ - center_z;
            double tracking_error = std::sqrt(pos_error_x*pos_error_x +
                                             pos_error_y*pos_error_y +
                                             pos_error_z*pos_error_z);

            RCLCPP_INFO(get_logger(),
                       "Circle: %.0f%% complete (%.1f°) | Tracking error: %.2fm | Cmd:[%.1f,%.1f,%.1f] Act:[%.1f,%.1f,%.1f]",
                       progress, angle * 180.0 / M_PI, tracking_error,
                       x_ned, y_ned, center_z, x_, y_, z_);
        }

        std::this_thread::sleep_for(dt);
    }

    // Hold final position for 1 second to ensure we complete the circle
    for (int i = 0; i < rate_hz && rclcpp::ok(); ++i) {
        // Send offboard control mode
        offboard_heartbeat_.timestamp = getTimestamp();
        offboard_mode_publisher_->publish(offboard_heartbeat_);

        // Send trajectory setpoint to hold position
        sendTrajectorySetpointPosition(
            static_cast<float>(center_x),
            static_cast<float>(center_y),
            static_cast<float>(center_z),
            static_cast<float>(start_heading)
        );
        std::this_thread::sleep_for(dt);
    }

    RCLCPP_INFO(get_logger(), "Circle flight completed!");

    // Restart the heartbeat if it was running before
    if (heartbeat_was_running) {
        RCLCPP_INFO(get_logger(), "Resuming offboard heartbeat");
        startOffboardHeartbeat();
    }
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
