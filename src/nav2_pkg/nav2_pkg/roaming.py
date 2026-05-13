#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class Roaming(Node):
    def __init__(self):
        super().__init__("roaming")
        
        self.subscriber_=self.create_subscription(LaserScan,"scan",self.publish_scan,10)
        self.publisher_=self.create_publisher(TwistStamped,"/cmd_vel",10)
        self.declare_parameter("distance",0.3)
        self.threshold=self.get_parameter("distance").value

        self.flag=False
    def publish_scan(self,msg:LaserScan):

        cmd=TwistStamped()
        cmd.header.stamp=self.get_clock().now().to_msg()
        cmd.header.frame_id="map"

        
        # if self.flag:
        #     return 
        self.scan=msg.ranges[337:360]+msg.ranges[0:337]
        self.windows=[]
        self.windows.append(self.scan[0:46])
        self.windows.append(self.scan[46:92])
        self.windows.append(self.scan[92:138])
        self.windows.append(self.scan[138:184])
        self.windows.append(self.scan[184:230])
        self.windows.append(self.scan[230:276])
        self.windows.append(self.scan[276:322])
        self.windows.append(self.scan[322:337])

        # print(len(self.windows))

        # for index,value in enumerate(self.scan):
        #     print(f"index:{index} Value:{value}")
        # self.flag=True

        if any(d<self.threshold for d in self.windows[0]):
            cmd.twist.linear.x=0.0
        else:
            cmd.twist.linear.x=0.25

        self.publisher_.publish(cmd)


        for windowIndex,window in enumerate(self.windows):
            for index,rays in enumerate(window):
                print(f"window index:{windowIndex} index:{index,rays}")

        


        
      


        






def main(args=None):
    rclpy.init(args=args)
    node=Roaming()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()

