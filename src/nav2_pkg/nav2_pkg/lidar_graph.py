#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import math

class WindowVisualizer(Node):
    def __init__(self):
        super().__init__("window_visualizer")

        # ---------- SUBSCRIPTIONS ----------
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.callback_scan, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.callback_odom, 10
        )

        # ---------- ROBOT STATE ----------
        self.robot_yaw = 0.0   # radians, updated from /odom

        plt.ion()
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, polar=True)
        self.window_size = 46
        self.colors = ["red","blue","green","orange","purple","brown","pink","black"]

    # ============================================================
    #  ODOMETRY CALLBACK — extract yaw from quaternion
    # ============================================================
    def callback_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # quaternion → yaw (rotation around Z axis)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)  # radians

    # ============================================================
    #  DRAW ROBOT ICON + ORIENTATION ARROW (rotates with yaw)
    # ============================================================
    def draw_robot(self, range_max, yaw_rad):
        ax = self.ax
        robot_size = range_max * 0.045

        body_w = robot_size * 1.8
        body_h = robot_size * 1.2

        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)

        def rotate(x, y):
            """Rotate a Cartesian point by robot yaw."""
            return x * cos_y - y * sin_y, x * sin_y + y * cos_y

        def cart_patch(patch_x, patch_y, w, h, **kwargs):
            """Return a FancyBboxPatch rotated and placed in polar-Cartesian space."""
            corners = [
                rotate(patch_x,     patch_y),
                rotate(patch_x + w, patch_y),
                rotate(patch_x + w, patch_y + h),
                rotate(patch_x,     patch_y + h),
            ]
            poly = plt.Polygon(corners, **kwargs)
            ax.add_patch(poly)

        # --- Body ---
        cart_patch(
            -body_w / 2, -body_h / 2, body_w, body_h,
            linewidth=2, edgecolor="black", facecolor="white",
            transform=ax.transData, zorder=10
        )

        # --- Left wheel ---
        cart_patch(
            -body_w / 2 - robot_size * 0.35, -body_h * 0.4,
            robot_size * 0.3, body_h * 0.8,
            linewidth=1.5, edgecolor="black", facecolor="gray",
            transform=ax.transData, zorder=10
        )

        # --- Right wheel ---
        cart_patch(
            body_w / 2 + robot_size * 0.05, -body_h * 0.4,
            robot_size * 0.3, body_h * 0.8,
            linewidth=1.5, edgecolor="black", facecolor="gray",
            transform=ax.transData, zorder=10
        )

        # --- Eyes (rotated with robot) ---
        for ex, ey in [(-body_w * 0.2, body_h * 0.15), (body_w * 0.2, body_h * 0.15)]:
            rx, ry = rotate(ex, ey)
            eye = plt.Circle(
                (rx, ry), robot_size * 0.15,
                color="blue", transform=ax.transData, zorder=11
            )
            ax.add_patch(eye)

        # --- Forward direction arrow ---
        # "Forward" in robot frame is +Y (body top), rotated by yaw
        arrow_start_r = body_h / 2 + robot_size * 0.1
        arrow_end_r   = arrow_start_r + range_max * 0.18

        # Convert to polar: angle in matplotlib polar = measured from East CCW
        # Our yaw=0 means North (up). Matplotlib polar angle 0 = East.
        # theta_zero_location="N" + theta_direction=1 (CW) is already set,
        # so we pass the angle directly as yaw (0 = North in our setup).
        forward_angle = yaw_rad  # in our polar frame, 0 rad = North

        ax.annotate(
            "",
            xy=(forward_angle, arrow_end_r),
            xytext=(forward_angle, arrow_start_r),
            arrowprops=dict(
                arrowstyle="-|>",
                color="cyan",
                lw=3,
                mutation_scale=20
            ),
            zorder=12
        )

        ax.text(
            forward_angle,
            arrow_end_r + range_max * 0.05,
            "FRONT",
            ha="center", va="center",
            fontsize=11, fontweight="bold",
            color="cyan", zorder=12
        )

    # ============================================================
    #  SCAN CALLBACK — rotate windows by robot yaw
    # ============================================================
    def callback_scan(self, msg: LaserScan):
        self.ax.clear()

        yaw = self.robot_yaw  # current robot heading in radians

        # ---------- FIX INF ----------
        fixed_scan = [
            msg.range_max if d == float("inf") else d
            for d in msg.ranges
        ]

        # ---------- REORDER (LiDAR zero = forward) ----------
        scan = fixed_scan[337:360] + fixed_scan[0:337]

        # ---------- CREATE WINDOWS ----------
        windows = []
        for i in range(0, len(scan), self.window_size):
            windows.append(scan[i:i + self.window_size])

        # ---------- WINDOW BOUNDARY ANGLES (rotated by yaw) ----------
        boundary_angles_deg = [337, 23, 69, 115, 161, 207, 253, 299]

        for angle_deg in boundary_angles_deg:
            # Rotate boundary by robot yaw
            rad = np.deg2rad(angle_deg) + yaw
            self.ax.plot(
                [rad, rad], [0, msg.range_max],
                color="gray", linewidth=2, linestyle="--"
            )
            self.ax.text(
                rad, msg.range_max + 0.2,
                f"{angle_deg}°",
                fontsize=10, fontweight="bold"
            )

        # ---------- PLOT WINDOWS (rotated by yaw) ----------
        current_angle = 337
        for index, window in enumerate(windows):
            angles = [
                np.deg2rad((current_angle + i) % 360) + yaw
                for i in range(len(window))
            ]
            self.ax.scatter(
                angles, window,
                color=self.colors[index % len(self.colors)],
                s=20
            )
            center_angle = (current_angle + len(window) // 2) % 360
            center_rad   = np.deg2rad(center_angle) + yaw
            avg_distance = sum(window) / len(window)
            self.ax.text(
                center_rad, avg_distance,
                f"W{index}",
                fontsize=14, fontweight="bold", color="black"
            )
            current_angle += len(window)

        # ---------- FRONT AXIS LINE (rotates with robot) ----------
        self.ax.plot(
            [yaw, yaw], [0, msg.range_max],
            color="cyan", linewidth=2, alpha=0.5,
            label="Robot front"
        )

        # ---------- DRAW ROBOT ICON ----------
        self.draw_robot(msg.range_max, yaw)

        # ---------- SETTINGS ----------
        self.ax.set_xticks([])
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(1)
        self.ax.set_title(
            f"WINDOW VISUALIZATION  |  yaw = {math.degrees(yaw):.1f}°",
            fontsize=16, pad=20
        )
        self.ax.set_rmax(msg.range_max)
        self.ax.grid(True)

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = WindowVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()