#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class PX4PosePublisher : public rclcpp::Node
{
public:
    PX4PosePublisher() : Node("px4_pose_publisher")
    {
        // QoS profile for PX4 messages
        auto qos_profile = rclcpp::QoS(1).best_effort().transient_local().keep_last(1);

        // Subscribe to PX4 vehicle odometry
        odometry_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos_profile,
            std::bind(&PX4PosePublisher::odometryCallback, this, std::placeholders::_1));

        // Publisher for ROS2 pose message
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/dexi/pose", 10);

        // TF broadcaster for visualization
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(get_logger(), "PX4 Pose Publisher initialized");
    }

private:
    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Convert PX4 odometry to ROS2 PoseStamped
        auto pose_msg = geometry_msgs::msg::PoseStamped();

        // Header
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";  // or "odom"

        // Position (PX4 uses NED, ROS uses ENU - convert if needed)
        pose_msg.pose.position.x = msg->position[0];   // North -> X
        pose_msg.pose.position.y = -msg->position[1];  // East -> -Y (NED to ENU)
        pose_msg.pose.position.z = -msg->position[2];  // Down -> -Z (NED to ENU)

        // Orientation (quaternion)
        pose_msg.pose.orientation.w = msg->q[0];
        pose_msg.pose.orientation.x = msg->q[1];
        pose_msg.pose.orientation.y = -msg->q[2];  // NED to ENU conversion
        pose_msg.pose.orientation.z = -msg->q[3];  // NED to ENU conversion

        // Publish pose
        pose_publisher_->publish(pose_msg);

        // Publish TF transform for visualization
        publishTransform(pose_msg);
    }

    void publishTransform(const geometry_msgs::msg::PoseStamped& pose)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header = pose.header;
        transform.child_frame_id = "base_link";  // Drone frame

        transform.transform.translation.x = pose.pose.position.x;
        transform.transform.translation.y = pose.pose.position.y;
        transform.transform.translation.z = pose.pose.position.z;

        transform.transform.rotation = pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4PosePublisher>());
    rclcpp::shutdown();
    return 0;
}