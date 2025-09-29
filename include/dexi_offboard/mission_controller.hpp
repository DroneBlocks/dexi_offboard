#ifndef DEXI_OFFBOARD__MISSION_CONTROLLER_HPP_
#define DEXI_OFFBOARD__MISSION_CONTROLLER_HPP_

#include <vector>
#include <queue>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"

struct Waypoint {
    double x, y, z, heading;
    std::string description;

    Waypoint(double x, double y, double z, double heading = 0.0, std::string desc = "")
        : x(x), y(y), z(z), heading(heading), description(desc) {}
};

class MissionController {
public:
    using TargetReachedCallback = std::function<void()>;
    using MissionCompleteCallback = std::function<void()>;

    MissionController();
    ~MissionController() = default;

    void addWaypoint(const Waypoint& waypoint);
    void addWaypoint(double x, double y, double z, double heading = 0.0, std::string desc = "");

    void startMission(TargetReachedCallback target_callback, MissionCompleteCallback complete_callback);
    void stopMission();
    void clearMission();

    bool isMissionActive() const { return mission_active_; }
    bool hasNextWaypoint() const { return !waypoints_.empty(); }

    Waypoint getNextWaypoint();
    void waypointReached();

    size_t remainingWaypoints() const { return waypoints_.size(); }

    void createBoxMission(double size, double current_x, double current_y, double current_z);

private:
    std::queue<Waypoint> waypoints_;
    bool mission_active_{false};
    Waypoint current_waypoint_{0, 0, 0, 0, "none"};

    TargetReachedCallback target_reached_callback_;
    MissionCompleteCallback mission_complete_callback_;
};

#endif // DEXI_OFFBOARD__MISSION_CONTROLLER_HPP_