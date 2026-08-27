#!/usr/bin/env python3
"""TCP client for real UGV collaboration.

This node runs on the ROS2 UAV computer. It connects to the ROS1
collab_ugv_tcp bridge on the AgileX LIMO, publishes the transformed UGV pose,
and optionally plans/follows a VLM path from the UAV birdview.
"""

import base64
import json
import math
import socket
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from uav_msgs.msg import ControlCommand

from tcp_protocol import message, recv_json, send_json


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class UgvCommunicateNode(Node):
    def __init__(self):
        super().__init__("ugv_communicate_node")
        self._declare_params()
        if not self._load_params():
            rclpy.shutdown()
            return

        self._lock = threading.Lock()
        self._socket_lock = threading.Lock()
        self._plan_lock = threading.Lock()
        self._stop = threading.Event()
        self._sock = None

        self._latest_odom = None
        self._latest_spf_goal = None
        self._latest_spf_detection = None
        self._latest_image = None
        self._latest_ugv_pose = None
        self._latest_ugv_reported = None
        self._last_ugv_state_time = 0.0
        self._last_missing_inputs = ""
        self._last_connection_log = 0.0
        self._path_seq = 0
        self._path_events = {}
        self._path_responses = {}

        if self.save_debug_images:
            self.debug_image_dir.mkdir(parents=True, exist_ok=True)

        self.ugv_pose_pub = self.create_publisher(PoseStamped, self.ugv_pose_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, 10
        )
        self.spf_goal_sub = self.create_subscription(
            ControlCommand, self.spf_goal_topic, self._spf_goal_cb, 10
        )
        self.spf_detection_sub = self.create_subscription(
            String, self.spf_detection_topic, self._spf_detection_cb, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_cb, qos_profile_sensor_data
        )

        self._tcp_thread = threading.Thread(target=self._tcp_loop, daemon=True)
        self._tcp_thread.start()
        self._decision_timer = self.create_timer(
            self.decision_interval_s, self._decision_tick
        )

        self.get_logger().info(
            "ugv_communicate_node: UGV TCP %s:%d planner=%s motion=%s"
            % (
                self.ugv_tcp_host,
                self.ugv_tcp_port,
                str(self.planner_enabled).lower(),
                str(self.enable_motion).lower(),
            )
        )

    def destroy_node(self):
        self._stop.set()
        self._stop_ugv()
        with self._socket_lock:
            sock = self._sock
            self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        super().destroy_node()

    def _declare_params(self):
        self.declare_parameter("ugv_tcp_host", "")
        self.declare_parameter("ugv_tcp_port", -1)
        self.declare_parameter("connect_timeout_s", -1.0)
        self.declare_parameter("connect_retry_s", -1.0)
        self.declare_parameter("ugv_state_timeout_s", -1.0)
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("spf_goal_topic", "")
        self.declare_parameter("spf_detection_topic", "")
        self.declare_parameter("ugv_pose_topic", "")
        self.declare_parameter("ugv_pose_frame_id", "")
        self.declare_parameter("downward_fov_deg", -1.0)
        self.declare_parameter("camera_yaw_offset_rad", math.nan)
        self.declare_parameter("min_projection_altitude_m", -1.0)
        self.declare_parameter("ugv_to_odom_x_m", math.nan)
        self.declare_parameter("ugv_to_odom_y_m", math.nan)
        self.declare_parameter("ugv_to_odom_z_m", math.nan)
        self.declare_parameter("ugv_to_odom_yaw_rad", math.nan)
        self.declare_parameter("annotated_jpeg_quality", -1)
        self.declare_parameter("planner_enabled", False)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("decision_interval_s", -1.0)
        self.declare_parameter("path_endpoint_tolerance_px", -1.0)
        self.declare_parameter("waypoint_tolerance_m", -1.0)
        self.declare_parameter("goal_tolerance_m", -1.0)
        self.declare_parameter("timeout_per_waypoint_s", -1.0)
        self.declare_parameter("control_period_s", -1.0)
        self.declare_parameter("stall_timeout_s", -1.0)
        self.declare_parameter("command_ttl_s", -1.0)
        self.declare_parameter("max_linear_speed_mps", -1.0)
        self.declare_parameter("max_angular_speed_rps", -1.0)
        self.declare_parameter("vlm_timeout_s", -1.0)
        self.declare_parameter("save_debug_images", False)
        self.declare_parameter("debug_image_dir", "")

    def _fail_param(self, name):
        self.get_logger().error("Parameter '%s' not set in YAML config" % name)
        return False

    def _load_required_string(self, name):
        value = str(self.get_parameter(name).value)
        if not value:
            self._fail_param(name)
            return None
        return value

    def _load_positive_float(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            self._fail_param(name)
            return None
        return value

    def _load_finite_float(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            self._fail_param(name)
            return None
        return value

    def _load_params(self):
        self.ugv_tcp_host = self._load_required_string("ugv_tcp_host")
        self.odom_topic = self._load_required_string("odom_topic")
        self.image_topic = self._load_required_string("image_topic")
        self.spf_goal_topic = self._load_required_string("spf_goal_topic")
        self.spf_detection_topic = self._load_required_string("spf_detection_topic")
        self.ugv_pose_topic = self._load_required_string("ugv_pose_topic")
        self.ugv_pose_frame_id = self._load_required_string("ugv_pose_frame_id")
        if None in (
            self.ugv_tcp_host,
            self.odom_topic,
            self.image_topic,
            self.spf_goal_topic,
            self.spf_detection_topic,
            self.ugv_pose_topic,
            self.ugv_pose_frame_id,
        ):
            return False

        self.ugv_tcp_port = int(self.get_parameter("ugv_tcp_port").value)
        if self.ugv_tcp_port <= 0 or self.ugv_tcp_port > 65535:
            return self._fail_param("ugv_tcp_port")

        self.connect_timeout_s = self._load_positive_float("connect_timeout_s")
        self.connect_retry_s = self._load_positive_float("connect_retry_s")
        self.ugv_state_timeout_s = self._load_positive_float("ugv_state_timeout_s")
        self.downward_fov_deg = self._load_positive_float("downward_fov_deg")
        self.min_projection_altitude_m = self._load_positive_float(
            "min_projection_altitude_m"
        )
        self.decision_interval_s = self._load_positive_float("decision_interval_s")
        self.path_endpoint_tolerance_px = self._load_positive_float(
            "path_endpoint_tolerance_px"
        )
        self.waypoint_tolerance_m = self._load_positive_float("waypoint_tolerance_m")
        self.goal_tolerance_m = self._load_positive_float("goal_tolerance_m")
        self.timeout_per_waypoint_s = self._load_positive_float(
            "timeout_per_waypoint_s"
        )
        self.control_period_s = self._load_positive_float("control_period_s")
        self.stall_timeout_s = self._load_positive_float("stall_timeout_s")
        self.command_ttl_s = self._load_positive_float("command_ttl_s")
        self.max_linear_speed_mps = self._load_positive_float("max_linear_speed_mps")
        self.max_angular_speed_rps = self._load_positive_float(
            "max_angular_speed_rps"
        )
        if None in (
            self.connect_timeout_s,
            self.connect_retry_s,
            self.ugv_state_timeout_s,
            self.downward_fov_deg,
            self.min_projection_altitude_m,
            self.decision_interval_s,
            self.path_endpoint_tolerance_px,
            self.waypoint_tolerance_m,
            self.goal_tolerance_m,
            self.timeout_per_waypoint_s,
            self.control_period_s,
            self.stall_timeout_s,
            self.command_ttl_s,
            self.max_linear_speed_mps,
            self.max_angular_speed_rps,
        ):
            return False
        if not 0.0 < self.downward_fov_deg < 180.0:
            return self._fail_param("downward_fov_deg")

        self.camera_yaw_offset_rad = self._load_finite_float("camera_yaw_offset_rad")
        self.ugv_to_odom_x_m = self._load_finite_float("ugv_to_odom_x_m")
        self.ugv_to_odom_y_m = self._load_finite_float("ugv_to_odom_y_m")
        self.ugv_to_odom_z_m = self._load_finite_float("ugv_to_odom_z_m")
        self.ugv_to_odom_yaw_rad = self._load_finite_float("ugv_to_odom_yaw_rad")
        if None in (
            self.camera_yaw_offset_rad,
            self.ugv_to_odom_x_m,
            self.ugv_to_odom_y_m,
            self.ugv_to_odom_z_m,
            self.ugv_to_odom_yaw_rad,
        ):
            return False

        self.annotated_jpeg_quality = int(
            self.get_parameter("annotated_jpeg_quality").value
        )
        if self.annotated_jpeg_quality < 1 or self.annotated_jpeg_quality > 100:
            return self._fail_param("annotated_jpeg_quality")

        self.planner_enabled = bool(self.get_parameter("planner_enabled").value)
        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.save_debug_images = bool(self.get_parameter("save_debug_images").value)
        self.debug_image_dir = Path(str(self.get_parameter("debug_image_dir").value))

        self.vlm_timeout_s = float(self.get_parameter("vlm_timeout_s").value)
        if self.vlm_timeout_s <= 0.0:
            return self._fail_param("vlm_timeout_s")

        return True

    def _odom_cb(self, msg):
        with self._lock:
            self._latest_odom = msg

    def _spf_goal_cb(self, msg):
        with self._lock:
            self._latest_spf_goal = msg

    def _spf_detection_cb(self, msg):
        try:
            detection = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn("Invalid SPF detection JSON: %s" % exc)
            return
        with self._lock:
            self._latest_spf_detection = detection

    def _image_cb(self, msg):
        with self._lock:
            self._latest_image = msg

    @staticmethod
    def _yaw_to_quat(yaw):
        return math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    @staticmethod
    def _yaw_from_odom(odom):
        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_from_pose(pose):
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _ugv_to_odom(self, x, y, z, yaw):
        cy = math.cos(self.ugv_to_odom_yaw_rad)
        sy = math.sin(self.ugv_to_odom_yaw_rad)
        odom_x = self.ugv_to_odom_x_m + x * cy - y * sy
        odom_y = self.ugv_to_odom_y_m + x * sy + y * cy
        odom_z = self.ugv_to_odom_z_m + z
        odom_yaw = wrap_angle(self.ugv_to_odom_yaw_rad + yaw)
        return odom_x, odom_y, odom_z, odom_yaw

    def _handle_ugv_state(self, state):
        odom_pose = state.get("odom_pose")
        if not isinstance(odom_pose, list) or len(odom_pose) < 3:
            return
        x, y, yaw = float(odom_pose[0]), float(odom_pose[1]), float(odom_pose[2])
        odom_x, odom_y, odom_z, odom_yaw = self._ugv_to_odom(x, y, 0.0, yaw)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.ugv_pose_frame_id
        pose.pose.position.x = odom_x
        pose.pose.position.y = odom_y
        pose.pose.position.z = odom_z
        pose.pose.orientation.z, pose.pose.orientation.w = self._yaw_to_quat(odom_yaw)
        self.ugv_pose_pub.publish(pose)

        with self._lock:
            self._latest_ugv_pose = pose
            self._latest_ugv_reported = {
                "x": x,
                "y": y,
                "z": 0.0,
                "yaw": yaw,
                "frame_id": "ugv_odom",
                "stamp": float(state.get("stamp", 0.0)),
                "twist": state.get("twist", {}),
                "image_seq": int(state.get("image_seq", -1)),
            }
            self._last_ugv_state_time = time.monotonic()

    def _handle_path_result(self, response):
        request_id = str(response.get("request_id", ""))
        if not request_id:
            self.get_logger().warn("Received path_result without request_id")
            return
        with self._lock:
            self._path_responses[request_id] = response
            event = self._path_events.get(request_id)
        if event is not None:
            event.set()

    def _tcp_loop(self):
        while not self._stop.is_set():
            sock = None
            try:
                sock = socket.create_connection(
                    (self.ugv_tcp_host, self.ugv_tcp_port),
                    timeout=self.connect_timeout_s,
                )
                sock.settimeout(None)
                with self._socket_lock:
                    self._sock = sock
                send_json(sock, message("hello", client="nav_drone", want_image=True))
                self.get_logger().info("Connected to UGV TCP bridge")

                while not self._stop.is_set():
                    response = recv_json(sock)
                    kind = response.get("type")
                    if kind == "hello_ack":
                        continue
                    if kind == "state":
                        self._handle_ugv_state(response)
                    elif kind == "path_result":
                        self._handle_path_result(response)
            except Exception as exc:
                now = time.monotonic()
                if now - self._last_connection_log > 5.0:
                    self.get_logger().warn("UGV TCP disconnected: %s" % exc)
                    self._last_connection_log = now
            finally:
                with self._socket_lock:
                    if self._sock is sock:
                        self._sock = None
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            if not self._stop.wait(self.connect_retry_s):
                continue

    def _send_to_ugv(self, payload):
        with self._socket_lock:
            sock = self._sock
            if sock is None:
                return False
            try:
                send_json(sock, payload)
                return True
            except Exception as exc:
                self.get_logger().warn("Failed to send UGV command: %s" % exc)
                return False

    def _request_ugv_path(self, annotated_img, projection):
        self._path_seq += 1
        request_id = "%d_%d" % (self.get_clock().now().nanoseconds, self._path_seq)
        event = threading.Event()
        with self._lock:
            self._path_events[request_id] = event
            self._path_responses.pop(request_id, None)

        payload = message(
            "plan_path",
            request_id=request_id,
            annotated_jpeg=self._jpeg_b64(annotated_img),
            projection=projection,
        )
        if not self._send_to_ugv(payload):
            with self._lock:
                self._path_events.pop(request_id, None)
                self._path_responses.pop(request_id, None)
            raise RuntimeError("Failed to send plan_path request to UGV")

        if not event.wait(self.vlm_timeout_s + 5.0):
            with self._lock:
                self._path_events.pop(request_id, None)
                self._path_responses.pop(request_id, None)
            raise RuntimeError("Timed out waiting for UGV path_result")

        with self._lock:
            response = self._path_responses.pop(request_id, None)
            self._path_events.pop(request_id, None)
        if response is None:
            raise RuntimeError("UGV path_result missing after event")
        if response.get("status") != "ok":
            raise RuntimeError("UGV path planner error: %s" % response.get("error", "unknown"))
        path = response.get("path")
        if not isinstance(path, list):
            raise RuntimeError("UGV path_result missing path")
        debug_dir = response.get("debug_dir")
        if debug_dir:
            self.get_logger().info("UGV path debug images: %s" % debug_dir)
        return [[float(p[0]), float(p[1])] for p in path]

    def _send_cmd_vel(self, linear_x, angular_z, linear_y=0.0):
        if not self.enable_motion:
            linear_x = 0.0
            linear_y = 0.0
            angular_z = 0.0
        linear_x = float(np.clip(linear_x, -self.max_linear_speed_mps,
                                 self.max_linear_speed_mps))
        linear_y = float(np.clip(linear_y, -self.max_linear_speed_mps,
                                 self.max_linear_speed_mps))
        angular_z = float(np.clip(angular_z, -self.max_angular_speed_rps,
                                  self.max_angular_speed_rps))
        return self._send_to_ugv(message(
            "cmd_vel",
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            ttl_s=self.command_ttl_s,
        ))

    def _stop_ugv(self):
        self._send_to_ugv(message("stop"))

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

    def _jpeg_b64(self, image_bgr):
        ok, buf = cv2.imencode(
            ".jpg", image_bgr,
            [cv2.IMWRITE_JPEG_QUALITY, self.annotated_jpeg_quality],
        )
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return base64.b64encode(buf.tobytes()).decode("ascii")

    @staticmethod
    def _clip_pixel(px, py, image_width, image_height):
        clipped_x = max(0, min(image_width - 1, int(round(px))))
        clipped_y = max(0, min(image_height - 1, int(round(py))))
        visible = 0 <= px < image_width and 0 <= py < image_height
        return {"x": clipped_x, "y": clipped_y, "visible": bool(visible)}

    def _world_to_pixel(self, world_x, world_y, odom, image_width, image_height):
        pos = odom.pose.pose.position
        altitude = max(float(pos.z), self.min_projection_altitude_m)
        half_span = altitude * math.tan(math.radians(self.downward_fov_deg / 2.0))
        if half_span <= 0.0:
            return None

        yaw = self._yaw_from_odom(odom) + self.camera_yaw_offset_rad
        wx = world_x - pos.x
        wy = world_y - pos.y
        body_forward = math.cos(yaw) * wx + math.sin(yaw) * wy
        body_left = -math.sin(yaw) * wx + math.cos(yaw) * wy

        dx_norm = -body_left / half_span
        dy_norm = -body_forward / half_span
        px = (dx_norm / 2.0 + 0.5) * image_width
        py = (dy_norm / 2.0 + 0.5) * image_height
        return self._clip_pixel(px, py, image_width, image_height)

    def _pixel_to_world(self, point, odom, image_width, image_height):
        px = float(point[0])
        py = float(point[1])
        pos = odom.pose.pose.position
        altitude = max(float(pos.z), self.min_projection_altitude_m)
        half_span = altitude * math.tan(math.radians(self.downward_fov_deg / 2.0))
        yaw = self._yaw_from_odom(odom) + self.camera_yaw_offset_rad

        dx_norm = (px / float(image_width) - 0.5) * 2.0
        dy_norm = (py / float(image_height) - 0.5) * 2.0
        body_forward = -dy_norm * half_span
        body_left = -dx_norm * half_span
        world_x = pos.x + body_forward * math.cos(yaw) - body_left * math.sin(yaw)
        world_y = pos.y + body_forward * math.sin(yaw) + body_left * math.cos(yaw)
        return np.array([world_x, world_y], dtype=float)

    def _detection_to_pixel(self, detection, image_width, image_height):
        if not isinstance(detection, dict):
            return None
        point = detection.get("point")
        if isinstance(point, list) and len(point) == 2:
            return self._clip_pixel(float(point[0]), float(point[1]),
                                    image_width, image_height)
        if "x" in detection and "y" in detection:
            return self._clip_pixel(float(detection["x"]), float(detection["y"]),
                                    image_width, image_height)
        return None

    def _build_annotation(self, image_msg, odom, ugv_pose, spf_detection):
        image_bgr = self._image_to_bgr(image_msg)
        image_height, image_width = image_bgr.shape[:2]
        center = (image_width // 2, image_height // 2)
        cv2.drawMarker(image_bgr, center, (0, 255, 0),
                       markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)

        projection = {"image_width": image_width, "image_height": image_height}
        ugv_px = None
        goal_px = None

        if ugv_pose is not None:
            p = ugv_pose.pose.position
            ugv_px = self._world_to_pixel(p.x, p.y, odom, image_width, image_height)
            projection["ugv_px"] = ugv_px
            if ugv_px is not None:
                cv2.circle(image_bgr, (ugv_px["x"], ugv_px["y"]), 12, (255, 0, 0), 2)
                cv2.putText(image_bgr, "UGV", (ugv_px["x"] + 10, ugv_px["y"] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)

        goal_px = self._detection_to_pixel(spf_detection, image_width, image_height)
        projection["goal_px"] = goal_px
        if goal_px is not None:
            cv2.circle(image_bgr, (goal_px["x"], goal_px["y"]), 12, (0, 0, 255), 2)
            cv2.putText(image_bgr, "GOAL", (goal_px["x"] + 10, goal_px["y"] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        if ugv_px is not None and goal_px is not None:
            cv2.line(image_bgr, (ugv_px["x"], ugv_px["y"]),
                     (goal_px["x"], goal_px["y"]), (0, 255, 255), 2)

        return image_bgr, projection

    def _validate_path(self, path, projection, image_width, image_height):
        if len(path) < 2:
            raise RuntimeError("UGV VLM path has fewer than two waypoints")
        for point in path:
            if len(point) != 2:
                raise RuntimeError("UGV VLM waypoint is not 2D")
            if not math.isfinite(point[0]) or not math.isfinite(point[1]):
                raise RuntimeError("UGV VLM waypoint is not finite")
            if point[0] < -image_width or point[0] > image_width * 2:
                raise RuntimeError("UGV VLM waypoint x is outside guard band")
            if point[1] < -image_height or point[1] > image_height * 2:
                raise RuntimeError("UGV VLM waypoint y is outside guard band")

        start = np.array(path[0], dtype=float)
        end = np.array(path[-1], dtype=float)
        ugv = np.array([projection["ugv_px"]["x"], projection["ugv_px"]["y"]],
                       dtype=float)
        goal = np.array([projection["goal_px"]["x"], projection["goal_px"]["y"]],
                        dtype=float)
        start_err = float(np.linalg.norm(start - ugv))
        end_err = float(np.linalg.norm(end - goal))
        if start_err > self.path_endpoint_tolerance_px:
            raise RuntimeError("UGV VLM path start error %.1f px" % start_err)
        if end_err > self.path_endpoint_tolerance_px:
            raise RuntimeError("UGV VLM path end error %.1f px" % end_err)

    def _current_ugv_pose(self):
        with self._lock:
            pose = self._latest_ugv_pose
            age = time.monotonic() - self._last_ugv_state_time
        if pose is None or age > self.ugv_state_timeout_s:
            raise RuntimeError("UGV state stale: %.2fs" % age)
        p = pose.pose.position
        return np.array([p.x, p.y], dtype=float), self._yaw_from_pose(pose)

    def _drive_towards(self, waypoint):
        pos, yaw = self._current_ugv_pose()
        dx, dy = waypoint[:2] - pos
        distance = math.hypot(dx, dy)
        bearing = wrap_angle(math.atan2(dy, dx) - yaw)

        if distance <= self.waypoint_tolerance_m:
            self._stop_ugv()
            return distance

        if abs(bearing) > math.radians(75.0):
            linear = 0.0
        else:
            linear = min(self.max_linear_speed_mps, max(0.05, 0.55 * distance))
            linear *= max(0.0, math.cos(bearing))
        angular = float(np.clip(1.8 * bearing,
                                -self.max_angular_speed_rps,
                                self.max_angular_speed_rps))
        self._send_cmd_vel(linear, angular)
        return distance

    def _follow_path(self, waypoints, goal_xy):
        for index, waypoint in enumerate(waypoints):
            started = time.monotonic()
            last_progress = started
            last_pos, _ = self._current_ugv_pose()
            while not self._stop.is_set():
                pos, _ = self._current_ugv_pose()
                if goal_xy is not None:
                    if float(np.linalg.norm(pos - goal_xy)) <= self.goal_tolerance_m:
                        self._stop_ugv()
                        return True

                distance = self._drive_towards(waypoint)
                if distance <= self.waypoint_tolerance_m:
                    break

                now = time.monotonic()
                if now - started > self.timeout_per_waypoint_s:
                    self.get_logger().warn(
                        "UGV waypoint %d timed out at %.2fm"
                        % (index + 1, distance)
                    )
                    self._stop_ugv()
                    break
                if float(np.linalg.norm(pos - last_pos)) > 0.08:
                    last_pos = pos.copy()
                    last_progress = now
                elif now - last_progress > self.stall_timeout_s:
                    self._stop_ugv()
                    raise RuntimeError("UGV appears stalled")

                time.sleep(self.control_period_s)

        self._stop_ugv()
        return False

    def _decision_tick(self):
        if not self.planner_enabled:
            return
        if self._plan_lock.locked():
            return
        thread = threading.Thread(target=self._plan_once, daemon=True)
        thread.start()

    def _missing_inputs(self, image_msg, odom, ugv_pose, spf_detection):
        missing = []
        if image_msg is None:
            missing.append("image")
        if odom is None:
            missing.append("uav_odom")
        if ugv_pose is None:
            missing.append("ugv_pose")
        if spf_detection is None:
            missing.append("spf_detection")
        return ",".join(missing)

    def _plan_once(self):
        if not self._plan_lock.acquire(blocking=False):
            return
        try:
            with self._lock:
                image_msg = self._latest_image
                odom = self._latest_odom
                ugv_pose = self._latest_ugv_pose
                spf_detection = self._latest_spf_detection

            missing = self._missing_inputs(image_msg, odom, ugv_pose, spf_detection)
            if missing:
                if missing != self._last_missing_inputs:
                    self.get_logger().warn("Waiting for UGV planner inputs: %s" % missing)
                    self._last_missing_inputs = missing
                return
            self._last_missing_inputs = ""

            annotated, projection = self._build_annotation(
                image_msg, odom, ugv_pose, spf_detection
            )
            ugv_px = projection.get("ugv_px")
            goal_px = projection.get("goal_px")
            if ugv_px is None or goal_px is None:
                self.get_logger().warn("Cannot project UGV or goal into UAV birdview")
                return
            if not ugv_px.get("visible", False) or not goal_px.get("visible", False):
                self.get_logger().warn("UGV or goal is outside UAV birdview")
                return

            if self.save_debug_images:
                stamp = self.get_clock().now().nanoseconds
                cv2.imwrite(str(self.debug_image_dir / ("ugv_birdview_%d.jpg" % stamp)),
                            annotated)

            path = self._request_ugv_path(annotated, projection)
            h, w = annotated.shape[:2]
            self._validate_path(path, projection, w, h)
            waypoints = [self._pixel_to_world(point, odom, w, h) for point in path]

            goal_xy = None
            if isinstance(spf_detection, dict) and isinstance(spf_detection.get("goal"), dict):
                goal = spf_detection["goal"]
                if "x" in goal and "y" in goal:
                    goal_xy = np.array([float(goal["x"]), float(goal["y"])], dtype=float)
            if goal_xy is None:
                goal_xy = waypoints[-1]

            if not self.enable_motion:
                self._stop_ugv()
                self.get_logger().info(
                    "UGV path planned (%d waypoints); motion disabled"
                    % len(waypoints)
                )
                return

            reached = self._follow_path(waypoints, goal_xy)
            self.get_logger().info("UGV follow_path finished reached=%s" % reached)
        except Exception as exc:
            self._stop_ugv()
            self.get_logger().warn("UGV planner failed: %s" % exc)
        finally:
            self._plan_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = UgvCommunicateNode()
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
