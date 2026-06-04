#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math


class Roaming(Node):
    def __init__(self):
        super().__init__("roaming")

        # ---------- pub / sub ----------
        self.subscriber_ = self.create_subscription(LaserScan, "scan",     self.publish_scan, 10)
        self.odom_sub_   = self.create_subscription(Odometry,  "/odom",    self.odom_cb,      10)
        self.publisher_  = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # ---------- parameters ----------
        self.declare_parameter("distance",     0.7)   # clearance threshold (m)
        self.declare_parameter("linear_speed", 0.25)  # forward speed  (m/s)
        self.declare_parameter("turn_speed",   0.5)   # rotation speed (rad/s)
        self.declare_parameter("angle_tol",    0.08)  # angle tolerance (rad) ~5 deg

        self.threshold    = self.get_parameter("distance").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.turn_speed   = self.get_parameter("turn_speed").value
        self.angle_tol    = self.get_parameter("angle_tol").value

        # ---------- state ----------
        self.current_yaw = 0.0      # latest yaw from odometry (rad)
        self.target_yaw  = None     # yaw we want to face      (rad)
        self.state       = "FORWARD"  # FORWARD | TURNING

        # ---------- window centre angles (robot frame, radians) ----------
        # Scan re-centred: index 0 = robot front, positive = left (CCW)
        # Windows: 0→0-46, 1→46-92, 2→92-138, 3→138-184,
        #          4→184-230, 5→230-276, 6→276-322, 7→322-337
        raw_centres_deg = [23, 69, 115, 161, 207, 253, 299, 329]
        self.window_centres = []
        for deg in raw_centres_deg:
            rad = math.radians(deg)
            if rad > math.pi:       # wrap behind-robot angles to negative
                rad -= 2 * math.pi
            self.window_centres.append(rad)

        self.get_logger().info("Roaming node started")

    # ------------------------------------------------------------------ #
    #  Odometry callback — extract yaw from quaternion                    #
    # ------------------------------------------------------------------ #
    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------ #
    #  Helpers                                                            #
    # ------------------------------------------------------------------ #
    def window_stats(self, window: list):
        """Return (avg, deviation_sum) for finite rays; (None, None) if empty."""
        valid = [r for r in window if math.isfinite(r)]
        if not valid:
            return None, None
        avg = sum(valid) / len(valid)
        dev = sum(abs(r - avg) for r in valid)
        return avg, dev

    def angle_diff(self, target: float, current: float) -> float:
        """Shortest signed angle from current to target (rad), range [-pi, pi]."""
        diff = target - current
        while diff >  math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return diff

    def publish_cmd(self, linear: float, angular: float):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = "map"
        cmd.twist.linear.x  = linear
        cmd.twist.angular.z = angular
        self.publisher_.publish(cmd)

    # ------------------------------------------------------------------ #
    #  Laser scan callback — main state machine                           #
    # ------------------------------------------------------------------ #
    def publish_scan(self, msg: LaserScan):

        # 1. Re-centre scan so index 0 = robot front
        scan = list(msg.ranges[337:360]) + list(msg.ranges[0:337])

        # 2. Build 8 windows (same split as your original code)
        windows = [
            scan[0:46],
            scan[46:92],
            scan[92:138],
            scan[138:184],
            scan[184:230],
            scan[230:276],
            scan[276:322],
            scan[322:337],
        ]

        # 3. Compute (avg, deviation_sum) for every window
        stats = [self.window_stats(w) for w in windows]
        front_avg, _ = stats[0]

        # ── STATE: TURNING ───────────────────────────────────────────────
        # Keep rotating until robot faces target_yaw
        if self.state == "TURNING":
            if self.target_yaw is None:
                self.state = "FORWARD"
                return

            diff = self.angle_diff(self.target_yaw, self.current_yaw)

            if abs(diff) < self.angle_tol:
                self.get_logger().info("Turn complete — resuming forward")
                self.publish_cmd(0.0, 0.0)
                self.state      = "FORWARD"
                self.target_yaw = None
            else:
                direction = 1.0 if diff > 0 else -1.0
                self.publish_cmd(0.0, direction * self.turn_speed)
            return

        # ── STATE: FORWARD ───────────────────────────────────────────────
        # Check if front window is blocked
        front_blocked = (
            (front_avg is not None and front_avg < self.threshold) or
            any(d < self.threshold for d in windows[0] if math.isfinite(d))
        )

        if not front_blocked:
            self.publish_cmd(self.linear_speed, 0.0)
            return

        # Front blocked — stop and choose best window to turn toward
        self.publish_cmd(0.0, 0.0)

        # 4. Score windows 1-7 (skip front window 0)
        #    Requirement : avg > threshold  (physically clear)
        #    Best window : highest avg AND lowest deviation
        #    Score       : avg - (dev_ratio * avg)  →  open flat corridor wins
        candidates = []
        for i in range(1, 8):
            avg, dev = stats[i]
            if avg is None or avg < self.threshold:
                continue
            candidates.append((i, avg, dev))

        if not candidates:
            # Completely surrounded — spin until a gap appears
            self.get_logger().warn("No clear window found — spinning")
            self.publish_cmd(0.0, self.turn_speed)
            return

        max_dev = max(c[2] for c in candidates) or 1.0
        best = max(candidates,
                   key=lambda c: c[1] - (c[2] / max_dev) * c[1])
        best_idx = best[0]

        # 5. Convert window centre angle (robot frame) → absolute yaw target
        relative_angle = self.window_centres[best_idx]
        target = self.current_yaw + relative_angle
        while target >  math.pi: target -= 2 * math.pi
        while target < -math.pi: target += 2 * math.pi

        self.target_yaw = target
        self.state      = "TURNING"

        self.get_logger().info(
            f"Blocked! Turning to window {best_idx} | "
            f"avg={best[1]:.2f}m  dev={best[2]:.2f} | "
            f"relative={math.degrees(relative_angle):.1f}deg  "
            f"target_yaw={math.degrees(target):.1f}deg"
        )


# ---------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = Roaming()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()











