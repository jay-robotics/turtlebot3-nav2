#!/usr/bin/env python3
"""ROS node for simple gap-based roaming behavior."""

import math
import random

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


class Roaming_2(Node):
    """Drive forward while choosing a safe gap to rotate toward."""

    def __init__(self):
        super().__init__("roaming_2")

        # Subscribe to laser scans and odometry.
        self.subscriber_ = self.create_subscription(
            LaserScan,
            "scan",
            self.publish_scan,
            10,
        )
        self.publisher_ = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            "/odom",
            self.get_heading,
            10,
        )

        # State used during gap selection and rotation.
        self.safe_gap_list = []
        self.motion_selector = "MOVING_FORWARD"
        self.selected_gap = None
        self.target_angle = None
        self.heading = 0.0
        self.merged = []
        self.fixed_selected_middle_ray = []
        self.closest_middle_ray = []
        self.select_from_whole_gap = 0
        self.select_from_chunks = 0

        # Parameters controlling the gap detection and movement behavior.
        self.declare_parameter("gap_threshold", 0.7)
        self.gap_threshold = self.get_parameter("gap_threshold").value

        print("Paramters: window:14 gap_width:14 obstacle:1.0")
        self.declare_parameter("window", 14)
        self.front_window_width = self.get_parameter("window").value

        self.declare_parameter("gap_width", 14)
        self.gap_width_threshold = self.get_parameter("gap_width").value

        self.declare_parameter("obstacle", 1.0)
        self.obstacle_threshold = self.get_parameter("obstacle").value

        self.declare_parameter("linear_speed", 1.0)
        self.linear_speed = self.get_parameter("linear_speed").value
        self.stop = 0.0
        self.angular_speed_slow = 0.05
        self.angular_speed_fast = 1.0
        self.error_speed_slow_threshold = 7
        self.error_threshold = 0.3

    def get_heading(self, msg: Odometry):
        """Convert the odometry quaternion to a heading in degrees."""
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        _, _, yaw = euler_from_quaternion([x, y, z, w])
        self.heading = math.degrees(yaw)  # -180 to 180
        if self.heading < 0:
            self.heading = self.heading + 360

    def publish_scan(self, msg: LaserScan, cmd: TwistStamped):
        """Main behavior loop: drive forward, find a safe gap, then rotate toward it."""
        counter = 0
        cmd = TwistStamped()
        self.cmd = cmd

        # Step 1: move forward while checking the front window for obstacles.
        if self.motion_selector == "MOVING_FORWARD":
            print("Moving Forward")

            # Build a front window from the laser scan data.
            if self.front_window_width % 2 == 0:
                self.rays = (self.front_window_width // 2)
                self.front_window = msg.ranges[0:self.rays] + msg.ranges[359:(359 - self.rays):-1]
            elif self.front_window_width % 2 != 0:
                self.rays = self.front_window_width // 2
                self.front_window = msg.ranges[0:self.rays] + msg.ranges[359:(359 - (self.front_window_width - self.rays)):-1]

            # Stop if two consecutive front-window rays detect an obstacle.
            for i in self.front_window:
                if i < self.obstacle_threshold or i == self.obstacle_threshold:
                    counter += 1

                    if counter == 2:
                        self.cmd.twist.linear.x = self.stop
                        self.publisher_.publish(self.cmd)
                        self.motion_selector = "FIND_BEST_GAP"
                        break
                elif i > self.obstacle_threshold:
                    counter = 0

            if counter < 2:
                self.cmd.twist.linear.x = self.linear_speed
                self.cmd.twist.angular.z = self.stop
                self.publisher_.publish(self.cmd)

        # Step 2: inspect the full scan and choose the best available gap.
        if self.motion_selector == "FIND_BEST_GAP":
            print("Finding best gap")

            self.gap_list = []
            self.safe_gap_list = []
            temp_list = []

            # Create a list of gaps from the laser scan.
            self.remaining = msg.ranges[0:360]
            for index, ray in enumerate(self.remaining):
                if math.isinf(ray):
                    ray = 3.5

                if ray > self.gap_threshold:
                    temp_list.append((index, ray))
                elif ray < self.gap_threshold:
                    if temp_list:
                        self.gap_list.append(temp_list)
                        temp_list = []

            if temp_list:
                self.gap_list.append(temp_list)

            # Keep only gaps that are wide enough to be considered safe.
            for index, gap in enumerate(self.gap_list):
                if len(gap) >= self.gap_width_threshold:
                    self.safe_gap_list.append(self.gap_list[index])

            # Merge gaps that wrap around the 0/359 boundary.
            self.merged = []
            if len(self.safe_gap_list) >= 2:
                if self.safe_gap_list[0][0][0] == 0 and self.safe_gap_list[-1][-1][0] == 359:
                    print(f"len safe gap list for merging:{len(self.safe_gap_list)} {self.safe_gap_list}")
                    self.merged = self.safe_gap_list[-1] + self.safe_gap_list[0]
                    self.safe_gap_list.pop(0)
                    self.safe_gap_list.pop(-1)
                    self.safe_gap_list.append(self.merged)

            # Find the average distance of each safe gap.
            self.total = 0
            self.average_list = []
            for gap in self.safe_gap_list:
                for ray_index, ray_distance in gap:
                    self.total = self.total + ray_distance
                    self.length = len(gap)
                self.average = self.total / self.length
                self.average_list.append(self.average)
                self.total = 0

            self.start_end_index = []
            for gap, average in zip(self.safe_gap_list, self.average_list):
                start = gap[0][0]
                end = gap[-1][0]
                # print(f"gap of width {len(gap)} start:{start} end:{end} has average distance {average} ")

            # Find the gap with the maximum average distance.
            self.maxValue = max(self.average_list)
            maxValue_index = self.average_list.index(self.maxValue)
            # print(f" max average value: {maxValue} at Index {maxValue_index}")
            self.start_safe_gap = self.safe_gap_list[maxValue_index][0][0]
            self.end_safe_gap = self.safe_gap_list[maxValue_index][-1][0]
            self.width = len(self.safe_gap_list[maxValue_index])
            safe_gap = self.safe_gap_list[maxValue_index]
            self.middle_ray_index = self.width // 2

            # Split safe gaps into chunks and pick a ray from the chunks.
            self.chunk_list1 = []
            self.temp_list1 = []
            for self.safe_gap in self.safe_gap_list:
                for ray, distance in self.safe_gap:
                    if len(self.temp_list1) < 14:
                        self.temp_list1.append(ray)
                    elif len(self.temp_list1) >= 14:
                        self.chunk_list1.append(self.temp_list1)
                        self.temp_list1 = []
                        self.temp_list1.append(ray)
                if len(self.temp_list1) < 14:
                    self.temp_list1 = []

            self.chunk_middle_ray_list = []
            for self.chunk in self.chunk_list1:
                middle_ray_index = len(self.chunk) // 2
                middle_ray = self.chunk[middle_ray_index]
                self.chunk_middle_ray_list.append(middle_ray)

            self.random_ray_from_chumks = random.choice(self.chunk_middle_ray_list)

            print(f"safe gap list:{self.safe_gap_list}\n")
            print(f"chunk list:{self.chunk_list1}\n")
            # print(f"len of gap:{len(self.gap_list)}\nlen of safe_gap_list:{len(self.safe_gap_list)}\nlen of chunk list;{len(self.chunk_list1)}\nlen of chunk_middle_ray_list:{len(self.chunk_middle_ray_list)}\nchunk_middle_ray_list:{self.chunk_middle_ray_list}\n random ray from chunks ray list:{self.random_ray_from_chumks}")

            # Select a middle ray from each safe gap and compare them.
            self.middle_ray_list = []
            print("enters second way")
            for self.gap in self.safe_gap_list:
                self.length = len(self.gap)
                self.middle_index = self.length // 2
                self.middleRay = self.gap[self.middle_index][0]
                self.middle_ray_list.append(self.middleRay)

            self.least_middle_ray = min(self.middle_ray_list)

            self.middle_ray = safe_gap[self.middle_ray_index]
            self.len_of_gaps = [len(i) for i in self.gap_list]
            self.len_gaps_in_safe_gap_list = {len(i) for i in self.safe_gap_list}
            # print(f"length of gap list:{(len(self.gap_list))} length of gaps in gap list:{self.len_of_gaps} \n length of safe_gap_list:{len(self.safe_gap_list)} len of gap in safe gap list:{self.len_gaps_in_safe_gap_list}  \n ength of middle gap list:{len(self.middle_ray_list)} Middle ray list:{self.middle_ray_list} selected ray:{self.least_middle_ray} min middle ray:{self.least_middle_ray}")
            self.fixed_selected_middle_ray.append(self.middle_ray[0])
            # print(f"fixed selected middle ray {self.fixed_selected_middle_ray}")
            # self.target=self.heading+self.middle_ray[0]

            # Randomly choose between the whole-gap middle ray and a chunk-based ray.
            self.random_selected_ray = random.choice([self.middle_ray[0], self.random_ray_from_chumks])

            if self.random_selected_ray == self.middle_ray[0]:
                self.select_from_whole_gap += 1
            elif self.random_selected_ray == self.random_ray_from_chumks:
                self.select_from_chunks += 1
            print(f"SELECTED {self.random_selected_ray}")
            print(f"number of times choosen: From whole gap:{self.select_from_whole_gap} From chunks:{self.select_from_chunks}")

            self.target = self.heading + self.random_selected_ray
            if self.target >= 360:
                self.target -= 360
            elif self.target <= 0:
                self.target += 360

            self.target_angle = self.target  # normalized 0,360
            self.motion_selector = "ROTATE"

        # Step 3: rotate toward the selected target angle.
        if self.motion_selector == "ROTATE":
            print("Rotating")

            # Normalize the heading error between -180 and 180.
            self.error = self.target_angle - self.heading
            if self.error > 180:
                self.error -= 360
            elif self.error < -180:
                self.error += 360

            if self.error < 0:
                if self.error < -self.error_speed_slow_threshold:
                    self.cmd.twist.angular.z = -self.angular_speed_fast
                    self.publisher_.publish(self.cmd)
                else:
                    self.cmd.twist.angular.z = -self.angular_speed_slow
            elif self.error > 0:
                if self.error <= self.error_speed_slow_threshold:
                    self.cmd.twist.angular.z = +self.angular_speed_slow
                    self.publisher_.publish(self.cmd)
                else:
                    self.cmd.twist.angular.z = +self.angular_speed_fast
                    self.publisher_.publish(self.cmd)
            else:
                self.cmd.twist.angular.z = 0.0
                self.publisher_.publish(self.cmd)

            print(f"Error {self.error}")
            print("\n")

            if -self.error_threshold < self.error < self.error_threshold:
                self.cmd.twist.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                print("Target reached | Moving Forward")
                self.motion_selector = "MOVING_FORWARD"


def main(args=None):
    rclpy.init(args=args)
    node = Roaming_2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
