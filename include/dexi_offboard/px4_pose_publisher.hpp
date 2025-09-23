#ifndef DEXI_OFFBOARD__PX4_POSE_PUBLISHER_HPP_
#define DEXI_OFFBOARD__PX4_POSE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class PX4PosePublisher : public rclcpp::Node
{
public:
    PX4PosePublisher();

private:
    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void publishTransform(const geometry_msgs::msg::PoseStamped& pose);

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // DEXI_OFFBOARD__PX4_POSE_PUBLISHER_HPP_