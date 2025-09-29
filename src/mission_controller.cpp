#include "dexi_offboard/mission_controller.hpp"
#include <cmath>

MissionController::MissionController() {
}

void MissionController::addWaypoint(const Waypoint& waypoint) {
    waypoints_.push(waypoint);
}

void MissionController::addWaypoint(double x, double y, double z, double heading, std::string desc) {
    waypoints_.emplace(x, y, z, heading, desc);
}

void MissionController::startMission(TargetReachedCallback target_callback, MissionCompleteCallback complete_callback) {
    if (waypoints_.empty()) {
        return;
    }

    target_reached_callback_ = target_callback;
    mission_complete_callback_ = complete_callback;
    mission_active_ = true;
}

void MissionController::stopMission() {
    mission_active_ = false;
}

void MissionController::clearMission() {
    while (!waypoints_.empty()) {
        waypoints_.pop();
    }
    mission_active_ = false;
}

Waypoint MissionController::getNextWaypoint() {
    if (waypoints_.empty()) {
        return Waypoint(0, 0, 0, 0, "empty");
    }

    current_waypoint_ = waypoints_.front();
    waypoints_.pop();
    return current_waypoint_;
}

void MissionController::waypointReached() {
    if (target_reached_callback_) {
        target_reached_callback_();
    }

    if (waypoints_.empty()) {
        mission_active_ = false;
        if (mission_complete_callback_) {
            mission_complete_callback_();
        }
    }
}

void MissionController::createBoxMission(double size, double current_x, double current_y, double current_z) {
    clearMission();

    // Box mission relative to current position: forward -> right -> backward -> left
    addWaypoint(current_x + size, current_y, current_z, 0, "Box: Forward");
    addWaypoint(current_x + size, current_y + size, current_z, 0, "Box: Right");
    addWaypoint(current_x, current_y + size, current_z, 0, "Box: Backward");
    addWaypoint(current_x, current_y, current_z, 0, "Box: Left (Home)");
}

