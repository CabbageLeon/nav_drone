#!/usr/bin/env python3
"""TCP bridge reserved for UGV collaboration.

Receives newline-delimited UGV pose JSON and returns latest UAV/SPF state JSON.
"""

import base64
import json
import math
import socket
import threading

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


class UgvCommunicateNode(Node):
    def __init__(self):
        super().__init__("ugv_communicate_node")
        self._declare_params()
        if not self._load_params():
            rclpy.shutdown()
            return

        self._lock = threading.Lock()
        self._latest_odom = None
        self._latest_spf_goal = None
        self._latest_spf_detection = None
        self._latest_image = None
        self._latest_ugv_pose = None
        self._latest_ugv_reported = None
        self._stop = threading.Event()
        self._server_socket = None

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

        self._server_thread = threading.Thread(target=self._tcp_loop, daemon=True)
        self._server_thread.start()
        self.get_logger().info(
            "ugv_communicate_node: listening on %s:%d" % (self.bind_address, self.tcp_port)
        )

    def destroy_node(self):
        self._stop.set()
        if self._server_socket is not None:
            try:
                self._server_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._server_socket.close()
            except OSError:
                pass
        super().destroy_node()

    def _declare_params(self):
        self.declare_parameter("tcp_bind_address", "")
        self.declare_parameter("tcp_port", -1)
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

    def _fail_param(self, name):
        self.get_logger().error("Parameter '%s' not set in YAML config" % name)
        return False

    def _load_finite_float(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            self._fail_param(name)
            return None
        return value

    def _load_params(self):
        self.bind_address = self.get_parameter("tcp_bind_address").value
        if not self.bind_address:
            return self._fail_param("tcp_bind_address")

        self.tcp_port = int(self.get_parameter("tcp_port").value)
        if self.tcp_port <= 0 or self.tcp_port > 65535:
            return self._fail_param("tcp_port")

        self.odom_topic = self.get_parameter("odom_topic").value
        if not self.odom_topic:
            return self._fail_param("odom_topic")

        self.image_topic = self.get_parameter("image_topic").value
        if not self.image_topic:
            return self._fail_param("image_topic")

        self.spf_goal_topic = self.get_parameter("spf_goal_topic").value
        if not self.spf_goal_topic:
            return self._fail_param("spf_goal_topic")

        self.spf_detection_topic = self.get_parameter("spf_detection_topic").value
        if not self.spf_detection_topic:
            return self._fail_param("spf_detection_topic")

        self.ugv_pose_topic = self.get_parameter("ugv_pose_topic").value
        if not self.ugv_pose_topic:
            return self._fail_param("ugv_pose_topic")

        self.ugv_pose_frame_id = self.get_parameter("ugv_pose_frame_id").value
        if not self.ugv_pose_frame_id:
            return self._fail_param("ugv_pose_frame_id")

        self.downward_fov_deg = float(self.get_parameter("downward_fov_deg").value)
        if not 0.0 < self.downward_fov_deg < 180.0:
            return self._fail_param("downward_fov_deg")

        self.camera_yaw_offset_rad = self._load_finite_float("camera_yaw_offset_rad")
        if self.camera_yaw_offset_rad is None:
            return False

        self.min_projection_altitude_m = float(
            self.get_parameter("min_projection_altitude_m").value
        )
        if self.min_projection_altitude_m <= 0.0:
            return self._fail_param("min_projection_altitude_m")

        self.ugv_to_odom_x_m = self._load_finite_float("ugv_to_odom_x_m")
        self.ugv_to_odom_y_m = self._load_finite_float("ugv_to_odom_y_m")
        self.ugv_to_odom_z_m = self._load_finite_float("ugv_to_odom_z_m")
        self.ugv_to_odom_yaw_rad = self._load_finite_float("ugv_to_odom_yaw_rad")
        if None in (
            self.ugv_to_odom_x_m,
            self.ugv_to_odom_y_m,
            self.ugv_to_odom_z_m,
            self.ugv_to_odom_yaw_rad,
        ):
            return False

        self.annotated_jpeg_quality = int(self.get_parameter("annotated_jpeg_quality").value)
        if self.annotated_jpeg_quality < 1 or self.annotated_jpeg_quality > 100:
            return self._fail_param("annotated_jpeg_quality")

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
        z = math.sin(yaw * 0.5)
        w = math.cos(yaw * 0.5)
        return z, w

    @staticmethod
    def _yaw_from_odom(odom):
        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _ugv_to_odom(self, x, y, z, yaw):
        cy = math.cos(self.ugv_to_odom_yaw_rad)
        sy = math.sin(self.ugv_to_odom_yaw_rad)
        odom_x = self.ugv_to_odom_x_m + x * cy - y * sy
        odom_y = self.ugv_to_odom_y_m + x * sy + y * cy
        odom_z = self.ugv_to_odom_z_m + z
        odom_yaw = self.ugv_to_odom_yaw_rad + yaw
        return odom_x, odom_y, odom_z, odom_yaw

    def _handle_ugv_line(self, line):
        data = json.loads(line)
        data = data.get("pose", data)
        x = float(data["x"])
        y = float(data["y"])
        z = float(data.get("z", 0.0))
        yaw = float(data.get("yaw", 0.0))
        odom_x, odom_y, odom_z, odom_yaw = self._ugv_to_odom(x, y, z, yaw)

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
                "z": z,
                "yaw": yaw,
                "frame_id": str(data.get("frame_id", "ugv")),
            }

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

    def _detection_to_pixel(self, detection, image_width, image_height):
        point = detection.get("point")
        if isinstance(point, list) and len(point) == 2:
            return self._clip_pixel(float(point[0]), float(point[1]), image_width, image_height)
        if "x" in detection and "y" in detection:
            return self._clip_pixel(float(detection["x"]), float(detection["y"]), image_width, image_height)
        return None

    def _annotation_payload(self, image_msg, odom, ugv_pose, spf_detection):
        if image_msg is None:
            return {}

        image_bgr = self._image_to_bgr(image_msg)
        image_height, image_width = image_bgr.shape[:2]
        center = (image_width // 2, image_height // 2)
        cv2.drawMarker(
            image_bgr, center, (0, 255, 0),
            markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2,
        )

        projection = {"image_width": image_width, "image_height": image_height}
        ugv_px = None
        goal_px = None

        if odom is not None and ugv_pose is not None:
            p = ugv_pose.pose.position
            ugv_px = self._world_to_pixel(p.x, p.y, odom, image_width, image_height)
            projection["ugv_px"] = ugv_px
            if ugv_px is not None:
                cv2.circle(image_bgr, (ugv_px["x"], ugv_px["y"]), 9, (255, 0, 0), 2)
                cv2.putText(
                    image_bgr, "UGV", (ugv_px["x"] + 10, ugv_px["y"] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2,
                )

        if spf_detection is not None:
            goal_px = self._detection_to_pixel(spf_detection, image_width, image_height)
            projection["goal_px"] = goal_px
            if goal_px is not None:
                cv2.circle(image_bgr, (goal_px["x"], goal_px["y"]), 10, (0, 0, 255), 2)
                cv2.putText(
                    image_bgr, "GOAL", (goal_px["x"] + 10, goal_px["y"] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2,
                )

        if ugv_px is not None and goal_px is not None:
            cv2.line(
                image_bgr,
                (ugv_px["x"], ugv_px["y"]),
                (goal_px["x"], goal_px["y"]),
                (0, 255, 255),
                2,
            )
            projection["ugv_to_goal_px"] = {
                "start": {"x": ugv_px["x"], "y": ugv_px["y"]},
                "end": {"x": goal_px["x"], "y": goal_px["y"]},
            }

        return {
            "projection": projection,
            "annotated_birdview": {
                "encoding": "jpg_base64",
                "width": image_width,
                "height": image_height,
                "data": self._jpeg_b64(image_bgr),
            },
        }

    def _state_response(self):
        with self._lock:
            odom = self._latest_odom
            spf_goal = self._latest_spf_goal
            spf_detection = self._latest_spf_detection
            image_msg = self._latest_image
            ugv_pose = self._latest_ugv_pose
            ugv_reported = self._latest_ugv_reported

        response = {"type": "uav_spf_state"}
        if odom is not None:
            p = odom.pose.pose.position
            response["uav"] = {
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "yaw": self._yaw_from_odom(odom),
                "frame_id": odom.header.frame_id,
            }
        if spf_goal is not None:
            p = spf_goal.pose.pose.position
            response["spf_goal"] = {
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "frame_id": spf_goal.pose.header.frame_id,
            }
        if spf_detection is not None:
            response["spf_detection"] = spf_detection
        if ugv_pose is not None:
            p = ugv_pose.pose.position
            response["last_ugv"] = {
                "x": p.x,
                "y": p.y,
                "z": p.z,
                "frame_id": ugv_pose.header.frame_id,
            }
        if ugv_reported is not None:
            response["last_ugv_reported"] = ugv_reported
        try:
            response.update(self._annotation_payload(image_msg, odom, ugv_pose, spf_detection))
        except Exception as exc:
            response["annotation_error"] = str(exc)
        return json.dumps(response, separators=(",", ":")) + "\n"

    def _client_loop(self, conn, addr):
        self.get_logger().info("UGV TCP client connected: %s:%d" % addr)
        pending = b""
        with conn:
            while not self._stop.is_set():
                chunk = conn.recv(4096)
                if not chunk:
                    break
                pending += chunk
                while b"\n" in pending:
                    raw_line, pending = pending.split(b"\n", 1)
                    line = raw_line.decode("utf-8").strip()
                    if not line:
                        continue
                    try:
                        self._handle_ugv_line(line)
                        conn.sendall(self._state_response().encode("utf-8"))
                    except Exception as exc:
                        err = {"type": "error", "message": str(exc)}
                        conn.sendall((json.dumps(err, separators=(",", ":")) + "\n").encode("utf-8"))
        self.get_logger().info("UGV TCP client disconnected: %s:%d" % addr)

    def _tcp_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket = srv
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.bind_address, self.tcp_port))
        srv.listen(1)
        srv.settimeout(0.5)
        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            thread = threading.Thread(target=self._client_loop, args=(conn, addr), daemon=True)
            thread.start()


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
