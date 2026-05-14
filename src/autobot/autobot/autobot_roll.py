#!/usr/bin/env python3
import math
import time
import threading
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False

OBSTACLE_DISTANCE_M = 1.0
CHASSIS_MIN_RANGE_M = 0.5
FORWARD_SPEED       = 0.2
FORWARD_CONE_DEG    = 90
STARTUP_DELAY_S     = 3.0
CAMERA_SAVE_DIR     = "/root/ros2_autobot/photos"

# Timed lawnmower coverage pattern. Tune these for your field and robot.
LAWNMOWER_ROWS          = 5
LAWNMOWER_ROW_TIME_S    = 8.0
LAWNMOWER_SHIFT_TIME_S  = 2.0
LAWNMOWER_TURN_90_S     = 2.0
LAWNMOWER_LINEAR_SPEED  = 0.2
LAWNMOWER_ANGULAR_SPEED = 0.55


class DualShockModeTeleop(Node):
    MODE_MANUAL = "manual"
    MODE_AUTO   = "auto"

    def __init__(self) -> None:
        super().__init__("dualshock_mode_teleop")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("cmd_vel_topic",   "/cmd_vel")
        self.declare_parameter("joy_topic",       "/joy")
        self.declare_parameter("mode_topic",      "/control_mode")
        self.declare_parameter("status_topic",    "/robot_status")
        self.declare_parameter("scan_topic",      "/scan")
        self.declare_parameter("max_linear",      2.00)
        self.declare_parameter("max_angular",     2.00)
        self.declare_parameter("deadzone",        0.10)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("btn_x",           0)
        self.declare_parameter("btn_circle",      1)
        self.declare_parameter("axis_left_x",     0)
        self.declare_parameter("axis_right_y",    3)
        self.declare_parameter("axis_l2",         4)
        self.declare_parameter("axis_r2",         5)
        self.declare_parameter("lawnmower_rows",          LAWNMOWER_ROWS)
        self.declare_parameter("lawnmower_row_time_s",    LAWNMOWER_ROW_TIME_S)
        self.declare_parameter("lawnmower_shift_time_s",  LAWNMOWER_SHIFT_TIME_S)
        self.declare_parameter("lawnmower_turn_90_s",     LAWNMOWER_TURN_90_S)
        self.declare_parameter("lawnmower_linear_speed",  LAWNMOWER_LINEAR_SPEED)
        self.declare_parameter("lawnmower_angular_speed", LAWNMOWER_ANGULAR_SPEED)

        self.max_linear      = float(self.get_parameter("max_linear").value)
        self.max_angular     = float(self.get_parameter("max_angular").value)
        self.deadzone        = float(self.get_parameter("deadzone").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.btn_x           = int(self.get_parameter("btn_x").value)
        self.btn_circle      = int(self.get_parameter("btn_circle").value)
        self.axis_left_x     = int(self.get_parameter("axis_left_x").value)
        self.axis_right_y    = int(self.get_parameter("axis_right_y").value)
        self.axis_l2         = int(self.get_parameter("axis_l2").value)
        self.axis_r2         = int(self.get_parameter("axis_r2").value)
        self.lawnmower_rows          = max(1, int(self.get_parameter("lawnmower_rows").value))
        self.lawnmower_row_time_s    = max(0.1, float(self.get_parameter("lawnmower_row_time_s").value))
        self.lawnmower_shift_time_s  = max(0.1, float(self.get_parameter("lawnmower_shift_time_s").value))
        self.lawnmower_turn_90_s     = max(0.1, float(self.get_parameter("lawnmower_turn_90_s").value))
        self.lawnmower_linear_speed  = float(self.get_parameter("lawnmower_linear_speed").value)
        self.lawnmower_angular_speed = float(self.get_parameter("lawnmower_angular_speed").value)

        # Robot state
        self._lock           = threading.Lock()
        self._camera_lock    = threading.Lock()
        self.mode            = self.MODE_MANUAL
        self.last_joy: Optional[Joy] = None
        self.lidar_ready     = False
        self.obstacle_ahead  = False
        self.auto_phase      = "IDLE"
        self.auto_start_time = None
        self.photo_count     = 0
        self.lawnmower_steps = []
        self.lawnmower_step_index = 0
        self.lawnmower_step_remaining = 0.0
        self.lawnmower_last_update = None
        self.lawnmower_obstacle_logged = False

        # Camera state
        self.camera_ready    = False
        self.pipeline        = None
        self.q_rgb           = None

        # ROS publishers/subscribers
        self.cmd_pub    = self.create_publisher(Twist,  "/cmd_vel",       10)
        self.mode_pub   = self.create_publisher(String, "/control_mode",  10)
        self.status_pub = self.create_publisher(String, "/robot_status",  10)

        self.create_subscription(Joy,       "/joy",  self.joy_callback,   10, callback_group=self.cb_group)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10, callback_group=self.cb_group)
        self.create_timer(1.0 / self.publish_rate_hz, self.control_loop,  callback_group=self.cb_group)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)

        # Camera setup — same API as working camera_node.py
        self._init_camera()

        self.get_logger().info("DualShock ready — MANUAL default")
        self.get_logger().info("X=AUTO(hold R2)  O=MANUAL(hold L2)")

    # ── Camera ───────────────────────────────────────────────────────────────

    def _init_camera(self):
        if not DEPTHAI_AVAILABLE:
            self._log("WARNING: depthai not installed — camera disabled")
            return

        try:
            os.makedirs(CAMERA_SAVE_DIR, exist_ok=True)

            # Exact same pipeline setup as working camera_node.py
            self.pipeline = dai.Pipeline()
            cam = self.pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A
            )
            video_out = cam.requestOutput(
                (1920, 1080),
                type=dai.ImgFrame.Type.BGR888p
            )
            self.q_rgb = video_out.createOutputQueue()
            self.pipeline.start()

            self.camera_ready = True
            self._log(f"OAK camera ready — saving photos to {CAMERA_SAVE_DIR}")

        except Exception as e:
            self.camera_ready = False
            self._log(f"WARNING: OAK camera failed to start: {e}")

    def _get_camera_frame(self):
        """Non-blocking frame grab — same as camera_node.py but uses tryGet."""
        if not self.camera_ready or self.q_rgb is None:
            return None
        try:
            in_rgb = self.q_rgb.tryGet()
            if in_rgb is None:
                return None
            return in_rgb.getCvFrame()
        except Exception as e:
            self._log(f"Camera frame read failed: {e}")
            return None

    def _capture_photo(self):
        """Called in background thread — captures and saves one photo."""
        with self._camera_lock:
            if not self.camera_ready:
                self._log("Obstacle stop — camera not ready, skipping photo")
                return

            # Try for up to 1 second to get a frame
            frame = None
            deadline = time.time() + 1.0
            while time.time() < deadline:
                frame = self._get_camera_frame()
                if frame is not None:
                    break
                time.sleep(0.05)

            if frame is None:
                self._log("Obstacle stop — no camera frame available")
                return

            # Publish to ROS so Foxglove can see it
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(msg)

            self.photo_count += 1
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(
                CAMERA_SAVE_DIR,
                f"obstacle_{self.photo_count:03d}_{timestamp}.jpg"
            )
            try:
                ok = cv2.imwrite(filename, frame)
                if ok:
                    self._log(f"Photo saved: {filename}")
                else:
                    self._log(f"WARNING: Failed to save photo: {filename}")
            except Exception as e:
                self._log(f"Photo save error: {e}")

    def _trigger_photo(self):
        """Fire-and-forget photo capture in background thread."""
        t = threading.Thread(target=self._capture_photo, daemon=True)
        t.start()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _log(self, msg):
        self.get_logger().info(msg)
        s = String(); s.data = msg
        self.status_pub.publish(s)

    def _send_cmd(self, lin=0.0, ang=0.0):
        t = Twist(); t.linear.x = lin; t.angular.z = ang
        self.cmd_pub.publish(t)

    def _stop(self):
        try: self._send_cmd(0.0, 0.0)
        except: pass

    def apply_deadzone(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def axis_pressed(self, joy, idx, threshold=0.5):
        return len(joy.axes) > idx and joy.axes[idx] < threshold

    def btn_down(self, joy, idx):
        return len(joy.buttons) > idx and joy.buttons[idx] == 1

    # ── LiDAR ─────────────────────────────────────────────────────────────────

    def lidar_callback(self, msg: LaserScan):
        with self._lock:
            if not self.lidar_ready:
                self.lidar_ready = True
                self._log(f"LiDAR ready — {len(msg.ranges)} rays")

        cone = math.radians(FORWARD_CONE_DEG)
        found = False
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > cone:               continue
            if math.isnan(r) or math.isinf(r): continue
            if r > msg.range_max:               continue
            if r < CHASSIS_MIN_RANGE_M:         continue
            if r < OBSTACLE_DISTANCE_M:
                found = True; break

        with self._lock:
            prev = self.obstacle_ahead
            self.obstacle_ahead = found
            mode  = self.mode
            phase = self.auto_phase

        if mode == self.MODE_AUTO:
            if found and not prev:
                self._log("OBSTACLE — stopping!")
                self._stop()
                self._trigger_photo()
            elif not found and prev and phase == "LAWNMOWER":
                self._log("Path clear - resuming")

    # ── Motion sequence ───────────────────────────────────────────────────────

    def _build_lawnmower_steps(self):
        steps = []
        for row in range(self.lawnmower_rows):
            steps.append((
                f"Row {row + 1}/{self.lawnmower_rows}",
                self.lawnmower_linear_speed,
                0.0,
                self.lawnmower_row_time_s,
                True,
            ))

            if row == self.lawnmower_rows - 1:
                break

            turn_dir = -1.0 if row % 2 == 0 else 1.0
            turn_name = "right" if turn_dir < 0.0 else "left"
            steps.append((
                f"Turn {turn_name}",
                0.0,
                turn_dir * self.lawnmower_angular_speed,
                self.lawnmower_turn_90_s,
                False,
            ))
            steps.append((
                "Shift to next lane",
                self.lawnmower_linear_speed,
                0.0,
                self.lawnmower_shift_time_s,
                True,
            ))
            steps.append((
                f"Turn {turn_name}",
                0.0,
                turn_dir * self.lawnmower_angular_speed,
                self.lawnmower_turn_90_s,
                False,
            ))
        return steps

    def _start_lawnmower_pattern(self):
        self.lawnmower_steps = self._build_lawnmower_steps()
        self.lawnmower_step_index = 0
        self.lawnmower_step_remaining = self.lawnmower_steps[0][3]
        self.lawnmower_last_update = time.time()
        self.lawnmower_obstacle_logged = False
        self._log(
            f"Starting lawnmower pattern: {self.lawnmower_rows} rows, "
            f"{self.lawnmower_row_time_s:.1f}s each"
        )
        self._log(self.lawnmower_steps[0][0])

    def _advance_lawnmower_step(self):
        self._stop()
        self.lawnmower_step_index += 1
        self.lawnmower_obstacle_logged = False

        if self.lawnmower_step_index >= len(self.lawnmower_steps):
            with self._lock:
                self.auto_phase = "DONE"
            self._log("=== Lawnmower pattern done ===")
            return

        label, _, _, dur, _ = self.lawnmower_steps[self.lawnmower_step_index]
        self.lawnmower_step_remaining = dur
        self.lawnmower_last_update = time.time()
        self._log(label)

    def _run_lawnmower_step(self, obstacle):
        if not self.lawnmower_steps:
            self._start_lawnmower_pattern()
            return

        label, lin, ang, _, stop_for_obstacle = self.lawnmower_steps[self.lawnmower_step_index]

        if obstacle and stop_for_obstacle:
            self._stop()
            self.lawnmower_last_update = time.time()
            if not self.lawnmower_obstacle_logged:
                self._log(f"Obstacle during {label} - paused")
                self.lawnmower_obstacle_logged = True
            return

        if self.lawnmower_obstacle_logged:
            self._log("Path clear - continuing lawnmower pattern")
            self.lawnmower_obstacle_logged = False

        now = time.time()
        if self.lawnmower_last_update is None:
            self.lawnmower_last_update = now
        dt = now - self.lawnmower_last_update
        self.lawnmower_last_update = now
        self.lawnmower_step_remaining -= dt

        if self.lawnmower_step_remaining <= 0.0:
            self._advance_lawnmower_step()
            return

        self._send_cmd(lin=lin, ang=ang)

    def _wait_lidar(self):
        self._log("Waiting for LiDAR...")
        deadline = time.time() + 10.0
        while True:
            with self._lock:
                if self.lidar_ready:              return True
                if self.mode == self.MODE_MANUAL: return False
            if time.time() > deadline:
                self._log("LiDAR timeout — continuing"); return True
            time.sleep(0.1)

    # ── Joy callback ──────────────────────────────────────────────────────────

    def joy_callback(self, msg: Joy):
        self.last_joy = msg
        if self.btn_down(msg, self.btn_circle):
            with self._lock:
                if self.mode != self.MODE_MANUAL:
                    self.mode = self.MODE_MANUAL
                    self.auto_phase = "IDLE"
                    self.lawnmower_steps = []
                    self.lawnmower_step_index = 0
                    self.lawnmower_step_remaining = 0.0
                    self.lawnmower_last_update = None
                    self._stop()
                    m = String(); m.data = self.mode
                    self.mode_pub.publish(m)
                    self._log("MANUAL mode")

        if self.btn_down(msg, self.btn_x):
            with self._lock:
                if self.mode != self.MODE_AUTO:
                    self.mode = self.MODE_AUTO
                    self.auto_phase = "WAITING"
                    self.auto_start_time = time.time()
                    self.lawnmower_steps = []
                    self.lawnmower_step_index = 0
                    self.lawnmower_step_remaining = 0.0
                    self.lawnmower_last_update = None
                    m = String(); m.data = self.mode
                    self.mode_pub.publish(m)
                    self._log("AUTO mode - hold R2, lawnmower starts in 3s...")

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        if self.last_joy is None:
            self._send_cmd(); return

        joy = self.last_joy

        if self.mode == self.MODE_MANUAL:
            if not self.axis_pressed(joy, self.axis_l2):
                self._send_cmd(); return
            lin = self.apply_deadzone(joy.axes[self.axis_right_y] if self.axis_right_y < len(joy.axes) else 0.0)
            ang = self.apply_deadzone(joy.axes[self.axis_left_x]  if self.axis_left_x  < len(joy.axes) else 0.0)
            self._send_cmd(self.max_linear * lin, self.max_angular * ang)
            return

        if self.mode == self.MODE_AUTO:
            if not self.axis_pressed(joy, self.axis_r2):
                self.lawnmower_last_update = time.time()
                self._send_cmd(); return

            with self._lock:
                phase    = self.auto_phase
                obstacle = self.obstacle_ahead

            if phase == "WAITING":
                with self._lock:
                    elapsed = time.time() - self.auto_start_time
                if elapsed >= STARTUP_DELAY_S:
                    if self._wait_lidar():
                        with self._lock:
                            self.auto_phase = "LAWNMOWER"
                        self._start_lawnmower_pattern()
                        self._log(f"Lawnmower active - pauses at {OBSTACLE_DISTANCE_M}m")

            elif phase == "LAWNMOWER":
                self._run_lawnmower_step(obstacle)

            elif phase == "DONE":
                self._stop()

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop()
        if self.pipeline is not None:
            try:
                self.pipeline.stop()  # same as working camera_node.py
            except Exception as e:
                self.get_logger().warn(f"Camera pipeline stop failed: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualShockModeTeleop()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == "__main__":
    main()
