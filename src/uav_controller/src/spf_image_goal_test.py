#!/usr/bin/env python3
"""Publish one local image as a down-facing camera stream and print SPF goals."""

import os

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from uav_msgs.msg import ControlCommand


class SpfImageGoalTest(Node):
    def __init__(self):
        super().__init__("spf_image_goal_test")
        self._declare_params()
        if not self._load_params():
            rclpy.shutdown()
            return

        self._image = cv2.imread(self.image_path, cv2.IMREAD_COLOR)
        if self._image is None:
            self.get_logger().error("Failed to read image: %s" % self.image_path)
            rclpy.shutdown()
            return

        self._last_goal_text = None
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.goal_sub = self.create_subscription(
            ControlCommand, self.goal_topic, self._goal_cb, 10
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_image)

        height, width = self._image.shape[:2]
        self.get_logger().info(
            "Publishing %s (%dx%d) to %s; waiting for %s"
            % (self.image_path, width, height, self.image_topic, self.goal_topic)
        )

    def _declare_params(self):
        self.declare_parameter("image_path", "")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("publish_rate_hz", -1.0)

    def _fail_param(self, name):
        self.get_logger().error("Parameter '%s' not set in YAML config" % name)
        return False

    def _load_params(self):
        spf_params = self._load_spf_params()
        if spf_params is None:
            return False

        self.image_topic = spf_params.get("image_topic", "")
        if not self.image_topic:
            return self._fail_param("spf_params.image_topic")

        self.goal_topic = spf_params.get("spf_goal_topic", "")
        if not self.goal_topic:
            return self._fail_param("spf_params.spf_goal_topic")

        self.image_path = self.get_parameter("image_path").value
        if not self.image_path:
            return self._fail_param("image_path")
        self.image_path = os.path.expanduser(self.image_path)
        if not os.path.isfile(self.image_path):
            self.get_logger().error("Image file not found: %s" % self.image_path)
            return False

        self.frame_id = self.get_parameter("frame_id").value
        if not self.frame_id:
            return self._fail_param("frame_id")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if self.publish_rate_hz <= 0.0:
            return self._fail_param("publish_rate_hz")

        return True

    def _load_spf_params(self):
        spf_params_path = os.path.join(
            get_package_share_directory("uav_controller"),
            "config",
            "spf_params.yaml",
        )
        if not os.path.isfile(spf_params_path):
            self.get_logger().error("SPF params file not found: %s" % spf_params_path)
            return None

        with open(spf_params_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        params = data.get("/spf_node", {}).get("ros__parameters", {})
        if not isinstance(params, dict):
            self.get_logger().error("Missing /spf_node.ros__parameters in %s" % spf_params_path)
            return None
        return params

    def _publish_image(self):
        height, width = self._image.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = self._image.tobytes()
        self.image_pub.publish(msg)

    def _goal_cb(self, msg):
        p = msg.pose.pose.position
        text = (
            "spf_goal: mode=%d frame=%s x=%.3f y=%.3f z=%.3f yaw=%.3f"
            % (msg.control_mode, msg.pose.header.frame_id, p.x, p.y, p.z, msg.yaw)
        )
        if text == self._last_goal_text:
            return
        self._last_goal_text = text
        print(text, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = SpfImageGoalTest()
    try:
        if rclpy.ok():
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
