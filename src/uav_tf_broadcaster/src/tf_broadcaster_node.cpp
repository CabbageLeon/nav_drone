/**
 * tf_broadcaster_node — TF2 tree manager + odometry relay
 *
 * What it does:
 *   - Publishes odom → base_link dynamic TF from bridge odometry
 *   - Publishes odom → base_link_stabilized TF (yaw-only, for 2D planning)
 *   - Publishes static TF for sensor extrinsics (base_link → camera/lidar/radar)
 *   - Relays bridge odometry to the system-wide /uav/odometry topic
 *
 * What it does NOT do:
 *   - State estimation or sensor fusion (not an estimator — future: separate EKF node)
 *
 * Frame tree:
 *   odom ──(dynamic)──► base_link ──(static)──► camera_link
 *                                    ├─(static)──► lidar_link
 *                                    └─(static)──► radar_link
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <string>
#include <vector>

class TfBroadcasterNode : public rclcpp::Node
{
public:
  TfBroadcasterNode() : Node("tf_broadcaster_node")
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/uav/odometry", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    bridge_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/uav/bridge/px4_odometry", 10,
        std::bind(&TfBroadcasterNode::odometry_cb, this, std::placeholders::_1));

    // All params use NaN as sentinel — YAML config MUST override them
    declare_parameter("sensors.camera.x", NAN);
    declare_parameter("sensors.camera.y", NAN);
    declare_parameter("sensors.camera.z", NAN);
    declare_parameter("sensors.camera.roll", NAN);
    declare_parameter("sensors.camera.pitch", NAN);
    declare_parameter("sensors.camera.yaw", NAN);
    declare_parameter("sensors.lidar.x", NAN);
    declare_parameter("sensors.lidar.y", NAN);
    declare_parameter("sensors.lidar.z", NAN);
    declare_parameter("sensors.radar.x", NAN);
    declare_parameter("sensors.radar.y", NAN);
    declare_parameter("sensors.radar.z", NAN);
    declare_parameter("sensors.radar.pitch", NAN);

    if (!validate_params())
    {
      RCLCPP_ERROR(get_logger(),
                   "tf_broadcaster_node: sensor calibration not configured. "
                   "Pass tf_broadcaster_params.yaml in launch file.");
      rclcpp::shutdown();
      return;
    }

    publish_static_transforms();
    RCLCPP_INFO(get_logger(), "tf_broadcaster_node started");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bridge_odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  bool validate_params()
  {
    const char *names[] = {
        "sensors.camera.x", "sensors.camera.y", "sensors.camera.z",
        "sensors.camera.roll", "sensors.camera.pitch", "sensors.camera.yaw",
        "sensors.lidar.x", "sensors.lidar.y", "sensors.lidar.z",
        "sensors.radar.x", "sensors.radar.y", "sensors.radar.z",
        "sensors.radar.pitch"};
    for (auto n : names)
    {
      if (std::isnan(get_parameter(n).as_double()))
      {
        RCLCPP_ERROR(get_logger(), "Parameter '%s' is not set — check YAML config", n);
        return false;
      }
    }
    return true;
  }

  void odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pub_->publish(*msg);

    // Dynamic TF: odom → base_link
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = msg->header.stamp;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      tf.transform.translation.x = msg->pose.pose.position.x;
      tf.transform.translation.y = msg->pose.pose.position.y;
      tf.transform.translation.z = msg->pose.pose.position.z;
      tf.transform.rotation = msg->pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // Stabilized frame (yaw only) — for 2D planning
    {
      const auto &q = msg->pose.pose.orientation;
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z));

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = msg->header.stamp;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link_stabilized";
      tf.transform.translation.x = msg->pose.pose.position.x;
      tf.transform.translation.y = msg->pose.pose.position.y;
      tf.transform.translation.z = msg->pose.pose.position.z;
      tf.transform.rotation.z = std::sin(yaw * 0.5);
      tf.transform.rotation.w = std::cos(yaw * 0.5);
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void publish_static_transforms()
  {
    auto now = get_clock()->now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    auto make_tf = [&](const std::string &child,
                       double x, double y, double z,
                       double roll, double pitch, double yaw)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = child;
      tf.transform.translation.x = x;
      tf.transform.translation.y = y;
      tf.transform.translation.z = z;
      double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
      double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
      double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
      tf.transform.rotation.w = cr * cp * cy + sr * sp * sy;
      tf.transform.rotation.x = sr * cp * cy - cr * sp * sy;
      tf.transform.rotation.y = cr * sp * cy + sr * cp * sy;
      tf.transform.rotation.z = cr * cp * sy - sr * sp * cy;
      return tf;
    };

    transforms.push_back(make_tf("camera_link",
        get_parameter("sensors.camera.x").as_double(),
        get_parameter("sensors.camera.y").as_double(),
        get_parameter("sensors.camera.z").as_double(),
        get_parameter("sensors.camera.roll").as_double(),
        get_parameter("sensors.camera.pitch").as_double(),
        get_parameter("sensors.camera.yaw").as_double()));

    transforms.push_back(make_tf("lidar_link",
        get_parameter("sensors.lidar.x").as_double(),
        get_parameter("sensors.lidar.y").as_double(),
        get_parameter("sensors.lidar.z").as_double(),
        0.0, 0.0, 0.0));

    transforms.push_back(make_tf("radar_link",
        get_parameter("sensors.radar.x").as_double(),
        get_parameter("sensors.radar.y").as_double(),
        get_parameter("sensors.radar.z").as_double(),
        0.0, get_parameter("sensors.radar.pitch").as_double(), 0.0));

    static_tf_broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(get_logger(), "Published %zu static sensor transforms", transforms.size());
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
