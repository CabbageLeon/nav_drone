/**
 * tf_broadcaster_node — TF2 tree manager + odometry relay + vision_pose bridge
 *
 * What it does:
 *   - Publishes odom → base_link dynamic TF from bridge odometry
 *   - Publishes odom → base_link_stabilized TF (yaw-only, for 2D planning)
 *   - Publishes static TF for sensor extrinsics (base_link → camera_link / body)
 *   - Relays bridge odometry to /uav/odometry
 *   - Subscribes to Fast-LIO /Odometry, looks up base_link→body from TF tree,
 *     applies inverse extrinsics, publishes /mavros/vision_pose/pose
 *
 * Frame tree:
 *   odom ──(dynamic)──► base_link ──(static)──► camera_link
 *                                    └─(static)──► body  (LIO output frame)
 *
 * Key: the static TF base_link→body is the SINGLE source of truth for sensor
 * calibration.  The vision_pose callback reads it from the TF buffer — same as
 * any other consumer — so changing the YAML updates everything automatically.
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include <cmath>
#include <string>
#include <vector>

class TfBroadcasterNode : public rclcpp::Node
{
public:
  TfBroadcasterNode() : Node("tf_broadcaster_node")
  {
    // ── Publishers ──
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/uav/odometry", 10);
    vision_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);

    // ── TF infrastructure ──
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    // ── Subscriptions ──
    bridge_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/uav/bridge/px4_odometry", 10,
        std::bind(&TfBroadcasterNode::odometry_cb, this, std::placeholders::_1));

    lio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10,
        std::bind(&TfBroadcasterNode::lio_odometry_cb, this, std::placeholders::_1));

    // ── Parameters (NaN sentinel — YAML MUST override) ──
    declare_parameter("sensors.camera.x", NAN);
    declare_parameter("sensors.camera.y", NAN);
    declare_parameter("sensors.camera.z", NAN);
    declare_parameter("sensors.camera.roll", NAN);
    declare_parameter("sensors.camera.pitch", NAN);
    declare_parameter("sensors.camera.yaw", NAN);
    declare_parameter("sensors.lidar.x", NAN);
    declare_parameter("sensors.lidar.y", NAN);
    declare_parameter("sensors.lidar.z", NAN);
    declare_parameter("sensors.lidar.roll", NAN);
    declare_parameter("sensors.lidar.pitch", NAN);
    declare_parameter("sensors.lidar.yaw", NAN);

    if (!validate_params())
    {
      RCLCPP_ERROR(get_logger(),
                   "tf_broadcaster_node: sensor calibration not configured. "
                   "Pass tf_broadcaster_params.yaml in launch file.");
      rclcpp::shutdown();
      return;
    }

    // Cache inverse extrinsics from params (same source as static TF)
    {
      static constexpr double D2R = M_PI / 180.0;
      tf2::Quaternion q;
      q.setRPY(get_parameter("sensors.lidar.roll").as_double() * D2R,
               get_parameter("sensors.lidar.pitch").as_double() * D2R,
               get_parameter("sensors.lidar.yaw").as_double() * D2R);
      tf2::Vector3 t(get_parameter("sensors.lidar.x").as_double(),
                     get_parameter("sensors.lidar.y").as_double(),
                     get_parameter("sensors.lidar.z").as_double());
      T_base_body_inv_ = tf2::Transform(q, t).inverse();

      // T_align: camera_init → true_world.  LIO world is tilted by the LiDAR
      // mounting angle (same physical rotation as the extrinsics), so we
      // rotate the frame by the same pitch/roll/yaw to align it with gravity.
      T_align_ = tf2::Transform(q);  // rotation only, zero translation
    }

    publish_static_transforms();
    RCLCPP_INFO(get_logger(), "tf_broadcaster_node started");
  }

private:
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bridge_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // Cached extrinsics inverse: (base_link → body)⁻¹
  tf2::Transform T_base_body_inv_;
  // LIO world alignment: camera_init is tilted by LiDAR mounting angle
  tf2::Transform T_align_;

  // ── Parameter validation ──
  bool validate_params()
  {
    const char *names[] = {
        "sensors.camera.x", "sensors.camera.y", "sensors.camera.z",
        "sensors.camera.roll", "sensors.camera.pitch", "sensors.camera.yaw",
        "sensors.lidar.x", "sensors.lidar.y", "sensors.lidar.z",
        "sensors.lidar.roll", "sensors.lidar.pitch", "sensors.lidar.yaw"};
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

  // ── Bridge odometry → dynamic TF + relay ──
  void odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pub_->publish(*msg);

    // TF: odom → base_link
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

    // TF: odom → base_link_stabilized (yaw-only)
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, tf2::getYaw(msg->pose.pose.orientation));

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = msg->header.stamp;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link_stabilized";
      tf.transform.translation.x = msg->pose.pose.position.x;
      tf.transform.translation.y = msg->pose.pose.position.y;
      tf.transform.translation.z = msg->pose.pose.position.z;
      tf.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // ── LIO odometry → vision_pose ──
  // T_base_body_ is built from the same YAML params as the static TF, so the
  // calibration lives in one place.  We read from params instead of the TF
  // buffer to avoid the race: a node cannot reliably receive its own static TF
  // before the first callback fires.  Other nodes still consume the static TF.
  void lio_odometry_cb(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // camera_init → body (from LIO)
    tf2::Transform T_cam_body;
    tf2::fromMsg(msg->pose.pose, T_cam_body);

    // Compose: true_world → base_link = (align) * (camera_init → body) * (body → base_link)
    tf2::Transform T_cam_base = T_align_ * T_cam_body * T_base_body_inv_;

    geometry_msgs::msg::PoseStamped vp;
    vp.header.stamp = msg->header.stamp;
    vp.header.frame_id = "map";
    tf2::toMsg(T_cam_base, vp.pose);
    vision_pose_pub_->publish(vp);
  }

  // ── Static sensor transforms (single source of truth) ──
  void publish_static_transforms()
  {
    auto now = get_clock()->now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    auto make_tf = [&](const std::string &child,
                       double x, double y, double z,
                       double roll_deg, double pitch_deg, double yaw_deg)
    {
      static constexpr double D2R = M_PI / 180.0;
      tf2::Quaternion q;
      q.setRPY(roll_deg * D2R, pitch_deg * D2R, yaw_deg * D2R);

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = child;
      tf.transform.translation.x = x;
      tf.transform.translation.y = y;
      tf.transform.translation.z = z;
      tf.transform.rotation = tf2::toMsg(q);
      return tf;
    };

    transforms.push_back(make_tf("camera_link",
        get_parameter("sensors.camera.x").as_double(),
        get_parameter("sensors.camera.y").as_double(),
        get_parameter("sensors.camera.z").as_double(),
        get_parameter("sensors.camera.roll").as_double(),
        get_parameter("sensors.camera.pitch").as_double(),
        get_parameter("sensors.camera.yaw").as_double()));

    // Published as base_link→body to match Fast-LIO's child_frame_id.
    // The vision_pose callback looks up this exact transform from the TF buffer.
    transforms.push_back(make_tf("body",
        get_parameter("sensors.lidar.x").as_double(),
        get_parameter("sensors.lidar.y").as_double(),
        get_parameter("sensors.lidar.z").as_double(),
        get_parameter("sensors.lidar.roll").as_double(),
        get_parameter("sensors.lidar.pitch").as_double(),
        get_parameter("sensors.lidar.yaw").as_double()));

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
