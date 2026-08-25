#!/usr/bin/env python3
"""SPF node for real-machine UAV deployment.

Downward image + odometry -> VLM target pixel -> odom-frame ControlCommand.
"""

import base64
import json
import math
import re
import threading
from pathlib import Path

import cv2
import httpx
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from openai import OpenAI
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from uav_msgs.msg import ControlCommand


class SpfNode(Node):
    def __init__(self):
        super().__init__("spf_node")
        self._declare_params()
        if not self._load_params():
            rclpy.shutdown()
            return

        self._latest_image = None
        self._latest_odom = None
        self._target_description = self.initial_target_description
        self._data_lock = threading.Lock()
        self._infer_lock = threading.Lock()
        self._missing_input_warned = False

        try:
            self._vlm_client = OpenAI(
                api_key=self.qwen_api_key,
                base_url=self.qwen_base_url,
                http_client=httpx.Client(proxy=None, trust_env=False),
                timeout=self.vlm_timeout_s,
            )
        except Exception as exc:
            self.get_logger().error("Failed to initialize VLM client: %s" % exc)
            rclpy.shutdown()
            return

        if self.save_debug_images:
            self.debug_image_dir.mkdir(parents=True, exist_ok=True)

        self.goal_pub = self.create_publisher(ControlCommand, self.spf_goal_topic, 10)
        self.detection_pub = self.create_publisher(String, self.spf_detection_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_cb, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, 10
        )
        self.target_description_sub = self.create_subscription(
            String, self.target_description_topic,
            self._target_description_cb, 10
        )
        self.timer = self.create_timer(self.decision_interval_s, self._decision_tick)

        self.get_logger().info(
            "spf_node: image=%s odom=%s goal=%s model=%s"
            % (self.image_topic, self.odom_topic, self.spf_goal_topic, self.qwen_model)
        )

    def _declare_params(self):
        self.declare_parameter("image_topic", "")
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("spf_goal_topic", "")
        self.declare_parameter("spf_detection_topic", "")
        self.declare_parameter("decision_interval_s", -1.0)
        self.declare_parameter("downward_fov_deg", -1.0)
        self.declare_parameter("camera_yaw_offset_rad", math.nan)
        self.declare_parameter("max_horizontal_step_m", -1.0)
        self.declare_parameter("height_step_m", -1.0)
        self.declare_parameter("min_altitude_m", -1.0)
        self.declare_parameter("max_altitude_m", -1.0)
        self.declare_parameter("qwen_model", "")
        self.declare_parameter("qwen_base_url", "")
        self.declare_parameter("qwen_api_key", "")
        self.declare_parameter("target_description", "")
        self.declare_parameter("target_description_topic", "")
        self.declare_parameter("goal_image_path", "")
        self.declare_parameter("vlm_temperature", -1.0)
        self.declare_parameter("vlm_max_tokens", -1)
        self.declare_parameter("vlm_timeout_s", -1.0)
        self.declare_parameter("save_debug_images", False)
        self.declare_parameter("debug_image_dir", "")

    def _fail_param(self, name):
        self.get_logger().error("Parameter '%s' not set in YAML config" % name)
        return False

    def _load_params(self):
        self.image_topic = self.get_parameter("image_topic").value
        if not self.image_topic:
            return self._fail_param("image_topic")

        self.odom_topic = self.get_parameter("odom_topic").value
        if not self.odom_topic:
            return self._fail_param("odom_topic")

        self.spf_goal_topic = self.get_parameter("spf_goal_topic").value
        if not self.spf_goal_topic:
            return self._fail_param("spf_goal_topic")

        self.spf_detection_topic = self.get_parameter("spf_detection_topic").value
        if not self.spf_detection_topic:
            return self._fail_param("spf_detection_topic")

        self.decision_interval_s = float(self.get_parameter("decision_interval_s").value)
        if self.decision_interval_s <= 0.0:
            return self._fail_param("decision_interval_s")

        self.downward_fov_deg = float(self.get_parameter("downward_fov_deg").value)
        if not 0.0 < self.downward_fov_deg < 180.0:
            return self._fail_param("downward_fov_deg")

        self.camera_yaw_offset_rad = float(self.get_parameter("camera_yaw_offset_rad").value)
        if not math.isfinite(self.camera_yaw_offset_rad):
            return self._fail_param("camera_yaw_offset_rad")

        self.max_horizontal_step_m = float(self.get_parameter("max_horizontal_step_m").value)
        if self.max_horizontal_step_m <= 0.0:
            return self._fail_param("max_horizontal_step_m")

        self.height_step_m = float(self.get_parameter("height_step_m").value)
        if self.height_step_m <= 0.0:
            return self._fail_param("height_step_m")

        self.min_altitude_m = float(self.get_parameter("min_altitude_m").value)
        if self.min_altitude_m < 0.0:
            return self._fail_param("min_altitude_m")

        self.max_altitude_m = float(self.get_parameter("max_altitude_m").value)
        if self.max_altitude_m <= self.min_altitude_m:
            return self._fail_param("max_altitude_m")

        self.qwen_model = self.get_parameter("qwen_model").value
        if not self.qwen_model:
            return self._fail_param("qwen_model")

        self.qwen_base_url = self.get_parameter("qwen_base_url").value
        if not self.qwen_base_url:
            return self._fail_param("qwen_base_url")

        self.qwen_api_key = self.get_parameter("qwen_api_key").value
        if not self.qwen_api_key:
            return self._fail_param("qwen_api_key")

        self.initial_target_description = self.get_parameter("target_description").value
        self.target_description_topic = self.get_parameter("target_description_topic").value
        if not self.target_description_topic:
            return self._fail_param("target_description_topic")

        goal_image_value = self.get_parameter("goal_image_path").value
        self.goal_image_path = Path(goal_image_value) if goal_image_value else None
        if self.goal_image_path and not self.goal_image_path.exists():
            self.get_logger().error("Goal image file not found: %s" % self.goal_image_path)
            return False

        self.vlm_temperature = float(self.get_parameter("vlm_temperature").value)
        if self.vlm_temperature < 0.0:
            return self._fail_param("vlm_temperature")

        self.vlm_max_tokens = int(self.get_parameter("vlm_max_tokens").value)
        if self.vlm_max_tokens <= 0:
            return self._fail_param("vlm_max_tokens")

        self.vlm_timeout_s = float(self.get_parameter("vlm_timeout_s").value)
        if self.vlm_timeout_s <= 0.0:
            return self._fail_param("vlm_timeout_s")

        self.save_debug_images = bool(self.get_parameter("save_debug_images").value)
        self.debug_image_dir = Path(self.get_parameter("debug_image_dir").value)
        if self.save_debug_images and not str(self.debug_image_dir):
            return self._fail_param("debug_image_dir")

        return True

    def _image_cb(self, msg):
        with self._data_lock:
            self._latest_image = msg

    def _odom_cb(self, msg):
        with self._data_lock:
            self._latest_odom = msg

    def _target_description_cb(self, msg):
        with self._data_lock:
            self._target_description = msg.data.strip()
        self.get_logger().info("Updated target description")

    def _decision_tick(self):
        if self._infer_lock.locked():
            return

        with self._data_lock:
            image_msg = self._latest_image
            odom_msg = self._latest_odom

        if image_msg is None or odom_msg is None:
            if not self._missing_input_warned:
                self.get_logger().warn("Waiting for image and odometry before SPF inference")
                self._missing_input_warned = True
            return
        self._missing_input_warned = False

        thread = threading.Thread(
            target=self._run_inference_once,
            args=(image_msg, odom_msg),
            daemon=True,
        )
        thread.start()

    def _run_inference_once(self, image_msg, odom_msg):
        if not self._infer_lock.acquire(blocking=False):
            return
        try:
            image_bgr = self._image_to_bgr(image_msg)
            if self.save_debug_images:
                stamp = self.get_clock().now().nanoseconds
                cv2.imwrite(str(self.debug_image_dir / ("spf_%d.jpg" % stamp)), image_bgr)
            result = self._call_vlm(image_bgr, image_msg.width, image_msg.height)
            self._publish_goal(result, odom_msg, image_msg.width, image_msg.height)
        except Exception as exc:
            self.get_logger().warn("SPF inference failed: %s" % exc)
        finally:
            self._infer_lock.release()

    def _image_to_bgr(self, msg):
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
            "mono8": 1,
        }
        if encoding not in channels_by_encoding:
            raise RuntimeError("Unsupported image encoding: %s" % msg.encoding)

        channels = channels_by_encoding[encoding]
        row_width = int(msg.width) * channels
        if msg.step < row_width:
            raise RuntimeError("Invalid image step: %d < %d" % (msg.step, row_width))

        data = np.frombuffer(msg.data, dtype=np.uint8)
        rows = []
        for row_idx in range(msg.height):
            start = row_idx * msg.step
            rows.append(data[start:start + row_width])
        arr = np.concatenate(rows).reshape((msg.height, msg.width, channels))

        if encoding == "bgr8":
            return arr.copy()
        if encoding == "rgb8":
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    @staticmethod
    def _jpeg_b64(image_bgr):
        ok, buf = cv2.imencode(".jpg", image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return base64.b64encode(buf.tobytes()).decode("ascii")

    def _call_vlm(self, image_bgr, width, height):
        with self._data_lock:
            target_description = self._target_description.strip()

        prompt = (
            "Find the red truck. Return ONLY valid JSON (no markdown, no explanation).\n"
            "Format: {\"point\": [x, y], \"height\": -1/0/1}\n"
            "x,y: pixel coordinates (0-1280, 0-960). height:-1=descend, 0=hold, 1=ascend.\n"
            "Always use height=-1 when truck is visible."
        )
        if target_description:
            prompt += "\nTarget description: " + target_description
        content = [{"type": "text", "text": prompt}]

        if self.goal_image_path:
            goal_img = cv2.imread(str(self.goal_image_path), cv2.IMREAD_COLOR)
            if goal_img is None:
                raise RuntimeError("Cannot read goal image: %s" % self.goal_image_path)
            content.insert(0, {
                "type": "image_url",
                "image_url": {"url": "data:image/jpeg;base64," + self._jpeg_b64(goal_img)},
            })

        content.append({
            "type": "image_url",
            "image_url": {"url": "data:image/jpeg;base64," + self._jpeg_b64(image_bgr)},
        })

        resp = self._vlm_client.chat.completions.create(
            model=self.qwen_model,
            temperature=self.vlm_temperature,
            max_tokens=self.vlm_max_tokens,
            messages=[{"role": "user", "content": content}],
        )
        text = resp.choices[0].message.content.strip()
        result = self._parse_vlm_json(text)

        point = result.get("point")
        if not isinstance(point, list) or len(point) != 2:
            raise RuntimeError("VLM response missing point: %s" % text[:160])

        x = float(point[0])
        y = float(point[1])
        height_cmd = max(-1, min(1, int(result.get("height", 0))))
        if not math.isfinite(x) or not math.isfinite(y):
            raise RuntimeError("VLM point is not finite")
        return x, y, height_cmd

    @staticmethod
    def _parse_vlm_json(text):
        if text.startswith("```"):
            text = text.split("\n", 1)[-1].rsplit("```", 1)[0].strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            match = re.search(r"\{.*\}", text, flags=re.S)
            if not match:
                raise
            return json.loads(match.group(0))

    @staticmethod
    def _yaw_from_odom(odom):
        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_goal(self, vlm_result, odom, image_width, image_height):
        px, py, height_cmd = vlm_result
        pos = odom.pose.pose.position
        yaw = self._yaw_from_odom(odom) + self.camera_yaw_offset_rad
        altitude = max(float(pos.z), self.min_altitude_m)
        half_span = altitude * math.tan(math.radians(self.downward_fov_deg / 2.0))

        dx_norm = (px / float(image_width) - 0.5) * 2.0
        dy_norm = (py / float(image_height) - 0.5) * 2.0

        body_forward = -dy_norm * half_span
        body_left = -dx_norm * half_span

        target_x = pos.x + body_forward * math.cos(yaw) - body_left * math.sin(yaw)
        target_y = pos.y + body_forward * math.sin(yaw) + body_left * math.cos(yaw)

        step_x = target_x - pos.x
        step_y = target_y - pos.y
        step_dist = math.hypot(step_x, step_y)
        if step_dist > self.max_horizontal_step_m:
            scale = self.max_horizontal_step_m / step_dist
            target_x = pos.x + step_x * scale
            target_y = pos.y + step_y * scale

        target_z = float(pos.z)
        if height_cmd < 0:
            target_z = max(self.min_altitude_m, target_z - self.height_step_m)
        elif height_cmd > 0:
            target_z = min(self.max_altitude_m, target_z + self.height_step_m)

        goal = ControlCommand()
        goal.control_mode = ControlCommand.MODE_POSITION
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "odom"
        goal.pose.pose.position.x = float(target_x)
        goal.pose.pose.position.y = float(target_y)
        goal.pose.pose.position.z = float(target_z)
        goal.pose.pose.orientation.w = 1.0
        goal.yaw = math.nan
        self.goal_pub.publish(goal)

        detection = {
            "point": [float(px), float(py)],
            "height": int(height_cmd),
            "image_width": int(image_width),
            "image_height": int(image_height),
            "goal": {
                "x": float(target_x),
                "y": float(target_y),
                "z": float(target_z),
                "frame_id": goal.pose.header.frame_id,
            },
        }
        detection_msg = String()
        detection_msg.data = json.dumps(detection, separators=(",", ":"))
        self.detection_pub.publish(detection_msg)

        self.get_logger().info(
            "SPF point=(%.0f,%.0f) h=%d -> odom(%.2f, %.2f, %.2f)"
            % (px, py, height_cmd, target_x, target_y, target_z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpfNode()
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
