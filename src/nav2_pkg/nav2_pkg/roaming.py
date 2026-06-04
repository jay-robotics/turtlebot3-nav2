#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math


class Roaming(Node):

    def __init__(self):
        super().__init__("roaming")

        self.subscriber_ = self.create_subscription(
            LaserScan,
            "scan",
            self.publish_scan,
            10
        )

        self.publisher_ = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            10
        )

        self.declare_parameter("distance", 0.7)
        self.threshold = self.get_parameter("distance").value

        self.flag = False

    def publish_scan(self, msg: LaserScan):

        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "map"

        # reorder scan
        self.scan = msg.ranges[337:360] + msg.ranges[0:337]

        # create windows
        self.windows = []

        self.windows.append(self.scan[0:46])
        self.windows.append(self.scan[46:92])
        self.windows.append(self.scan[92:138])
        self.windows.append(self.scan[138:184])
        self.windows.append(self.scan[184:230])
        self.windows.append(self.scan[230:276])
        self.windows.append(self.scan[276:322])
        self.windows.append(self.scan[322:337])

        # store averages and deviations
        self.deviations = []
        self.averageList = [] 

        for window in self.windows[1:8]:

            valid_window = []

            for ray in window:

                if math.isinf(ray) or math.isnan(ray):
                    valid_window.append(3.5)
                else:
                    valid_window.append(ray)

            if len(valid_window) == 0:
                continue

            average = sum(valid_window) / len(valid_window)
            self.averageList.append(average)

            tempList = []

            for ray in valid_window:
                deviation = abs(ray - average)
                tempList.append(deviation)
            self.deviations.append(tempList)

        # sum deviations   small value=cleaner corridor,large value=obstacles
        self.sum_of_deviatons = []  
        for deviationsList in self.deviations:

            add = sum(deviationsList)
            self.sum_of_deviatons.append(add)

        # middle angles
        middle_angle_list = []
        for index, w in enumerate(self.windows):
            window_start = (337 + (index * 46)) % 360
            middle_ray = len(w) // 2
            middle_angle = (window_start + middle_ray) % 360
            middle_angle_list.append(middle_angle)



        
        self.publisher_.publish(cmd)






              
             
            
            






def main(args=None):

    rclpy.init(args=args)
    node = Roaming()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()



