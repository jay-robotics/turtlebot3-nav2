#!/usr/bin/env python3
"""ROS node for semantic mapping and object localization using YOLO."""

import json
import math
import os
from pathlib import Path

# Set up QT fonts before importing OpenCV.
for font_dir in (
    "/usr/share/fonts/truetype/dejavu",
    "/usr/share/fonts/truetype/liberation",
    "/usr/share/fonts",
):
    if os.path.isdir(font_dir):
        os.environ.setdefault("QT_QPA_FONTDIR", font_dir)
        break

import cv2
import rclpy
import tf2_geometry_msgs
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

# Ensure Qt fonts are available for OpenCV.
cv2_root = Path(cv2.__file__).resolve().parent
qt_fonts_dir = cv2_root / "qt" / "fonts"
qt_fonts_dir.mkdir(parents=True, exist_ok=True)

if not any(qt_fonts_dir.iterdir()):
    try:
        os.symlink("/usr/share/fonts/truetype/dejavu", qt_fonts_dir / "dejavu")
    except FileExistsError:
        pass


class sematic_mapping(Node):
    """Detect objects using YOLO and localize them in the map frame."""

    def __init__(self):
        super().__init__("sematic_mapping")

        # Initialize the CV bridge and load the YOLO model.
        self.bridge = CvBridge()
        self.model = YOLO("yolo11x.pt")

        # Subscribe to synchronized image and laser scan messages.
        self.image_sub = Subscriber(self, Image, "/camera/image_raw")
        self.scan_sub = Subscriber(self, LaserScan, "/scan")
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.scan_sub],
            queue_size=10,
            slop=0.1,
        )
        self.ts.registerCallback(self.callback)

        # Initialize the TF listener and navigator.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active(localizer="slam_toolbox")

        # Angle list for matching image regions to laser scan rays.
        self.angle_list = [27, 18, 11, 8, 1, 358, 351, 348, 341, 331]

        # State tracking: detected objects, confirmed objects, and their positions.
        self.obj_names_set = set()  # All unique object names detected
        self.confirmed_objects = set()  # Objects with real (non-infinite) laser distances
        self.obj_w_distances_dict = {}  # object_name -> {map: (x, y), original: (x, y)}

        # Navigation state variables.
        self.send_goal = "idle"  # idle | send_goal | goal_sent
        self.target_object = None  # Currently targeted object for navigation
        self.nav_state = "IDLE"  # IDLE | NAVIGATING | GOAL_REACHED | GOAL_FAILED | etc

        # Publishers for the dashboard.
        # Publishes list of unique detected object names as JSON.
        self.objects_pub = self.create_publisher(String, "/semantic/objects", 10)

        # Publishes navigation status as JSON {state, distance_remaining, eta, x, y}.
        self.status_pub = self.create_publisher(String, "/semantic/nav_status", 10)

        # Publishes YOLO-annotated frame as JPEG for browser display.
        self.image_pub = self.create_publisher(
            CompressedImage,
            "/semantic/image/compressed",
            10,
        )

        # Subscribers for commands from the dashboard.
        self.goal_object_sub = self.create_subscription(
            String,
            "/semantic/goal_object",
            self.goal_object_callback,
            10,
        )

        self.cancel_sub = self.create_subscription(
            Bool,
            "/semantic/cancel",
            self.cancel_callback,
            10,
        )

        # Timer for publishing navigation status updates.
        self.status_timer = self.create_timer(0.5, self.publish_nav_status)

    def goal_object_callback(self, msg: String):
        """Handle a goal object selection from the dashboard."""
        object_name = msg.data
        if object_name not in self.obj_w_distances_dict:
            self.nav_state = "OBJECT_NOT_FOUND"
            self.get_logger().warn(f"Object '{object_name}' not in semantic map yet")
            return
        self.target_object = object_name
        self.send_next_goal(object_name)

    def send_next_goal(self, object_name):
        """Send a navigation goal to the object's map coordinates."""
        x_map, y_map = self.obj_w_distances_dict[object_name]["map"]
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x_map
        goal.pose.position.y = y_map
        goal.pose.orientation.w = 1.0

        self.navigator.goToPose(goal)
        self.send_goal = "goal_sent"
        self.nav_state = "NAVIGATING"

    def cancel_callback(self, msg: Bool):
        """Cancel the current navigation goal."""
        if msg.data:
            self.navigator.cancelTask()
            self.send_goal = "idle"
            self.nav_state = "GOAL_CANCELLED"

    def publish_nav_status(self):
        """Publish current navigation status (distance, ETA, position) to dashboard."""
        distance_remaining = None
        eta_seconds = None

        if self.send_goal == "goal_sent":
            if not self.navigator.isTaskComplete():
                # Goal in progress: publish feedback.
                feedback = self.navigator.getFeedback()
                if feedback:
                    distance_remaining = float(feedback.distance_remaining)
                    eta_seconds = float(feedback.estimated_time_remaining.sec)
            else:
                # Goal completed: check result.
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    if (
                        self.target_object
                        and self.target_object not in self.confirmed_objects
                        and self.target_object in self.obj_w_distances_dict
                    ):
                        # If object not confirmed yet, try again.
                        self.send_next_goal(self.target_object)
                    else:
                        self.nav_state = "GOAL_REACHED"
                        self.send_goal = "idle"
                elif result == TaskResult.CANCELED:
                    self.nav_state = "GOAL_CANCELLED"
                    self.send_goal = "idle"
                else:
                    self.nav_state = "GOAL_FAILED"
                    self.send_goal = "idle"

        # Get current robot position.
        x, y = None, None
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
        except Exception:
            pass

        payload = {
            "state": self.nav_state,
            "distance_remaining": distance_remaining,
            "eta_seconds": eta_seconds,
            "x": x,
            "y": y,
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def callback(self, img_msg: Image, scan_msg: LaserScan):
        """Process synchronized image and laser scan messages."""
        self.ranges = scan_msg.ranges
        self.frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        # Run YOLO inference on the frame.
        results = self.model(self.frame, conf=0.9, verbose=False)

        annotated_frame = self.frame
        for result in results:
            annotated_frame = result.plot()

            # For each detected object in the frame.
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                box_centrex = int((x1 + x2) / 2)
                class_id = int(box.cls[0])
                object_name = result.names[class_id]

                # Partition the frame horizontally to map image regions to laser angles.
                height, width = annotated_frame.shape[:2]
                centre_x = width // 2
                parts = 5
                step = (width // 2) // parts
                partitions = sorted(
                    {centre_x - i * step for i in range(parts)}
                    | {centre_x + i * step for i in range(parts)}
                )
                edge = [0] + partitions + [width]

                # Find which region the object's center falls into.
                for a in range(len(edge) - 1):
                    if edge[a] <= box_centrex < edge[a + 1]:
                        # Look up the laser ray that corresponds to this region.
                        theta = math.radians(self.angle_list[a])
                        raw_ray = self.ranges[self.angle_list[a]]

                        # If the ray is infinite, estimate distance; otherwise use the real one.
                        if math.isinf(raw_ray):
                            ray = scan_msg.range_max - 0.7
                            is_confirmed = False
                        else:
                            ray = raw_ray - 0.7
                            is_confirmed = True

                        # Convert polar to Cartesian coordinates in the base_scan frame.
                        px = ray * math.cos(theta)
                        py = ray * math.sin(theta)

                        point = PointStamped()
                        point.header.frame_id = "base_scan"
                        point.header.stamp = scan_msg.header.stamp
                        point.point.x = px
                        point.point.y = py
                        point.point.z = 0.0

                        # Transform the point to the map frame.
                        try:
                            tf = self.tf_buffer.lookup_transform(
                                "map",
                                "base_scan",
                                rclpy.time.Time.from_msg(scan_msg.header.stamp),
                                timeout=rclpy.duration.Duration(seconds=0.2),
                            )
                            point_map = tf2_geometry_msgs.do_transform_point(point, tf)
                            x_map = int(point_map.point.x * 2) / 2
                            y_map = int(point_map.point.y * 2) / 2

                            # Store the object's map coordinates.
                            self.obj_w_distances_dict[object_name] = {
                                "map": (x_map, y_map),
                                "original": (point_map.point.x, point_map.point.y),
                            }
                            if is_confirmed:
                                self.confirmed_objects.add(object_name)

                            # Announce new objects to the dashboard.
                            if object_name not in self.obj_names_set:
                                self.obj_names_set.add(object_name)
                                self.objects_pub.publish(
                                    String(data=json.dumps(sorted(self.obj_names_set)))
                                )
                        except Exception:
                            pass

        # Publish the annotated frame as JPEG for browser display.
        success, encoded = cv2.imencode(".jpg", annotated_frame)
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = sematic_mapping()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
