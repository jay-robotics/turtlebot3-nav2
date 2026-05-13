#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

class LaserscanNode(Node):
    def __init__(self):
        super().__init__("laserscan_node")

        self.subscriber_=self.create_subscription(LaserScan,"scan",self.publish_cmd,10)

        self.publisher_=self.create_publisher(TwistStamped,"/cmd_vel",10)

        self.declare_parameter("distance",1.0)
        self.distance=self.get_parameter("distance").value
        # self.timer_=self.create_timer(1.0,self.publish_cmd)
    
    # def callback_laserscan(self,msg:LaserScan):
    #     self.scan=msg.ranges
    #     a=self.scan[0:46]
    #     b=[]
    #     for i in range(len(a)):
    #         if a[i]<1:
    #             self.stop_cmd()
    #             # b.append(a[i])
                   
            
        # print(a)
        # print(len(a))
        # print(b)
        # print(len(b))

    # def publish_cmd(self,scan:LaserScan):

        # forward=TwistStamped()
        # forward.header.stamp=self.get_clock().now().to_msg()
        # forward.header.frame_id="map"
        # forward.twist.linear.x=0.26

        # stop=TwistStamped()
        # stop.header.stamp=self.get_clock().now().to_msg()
        # stop.header.frame_id="map"
        # stop.twist.linear.x=0.0

        
        # self.scan=scan.ranges                                                                                                             
        # a=self.scan[0:46]
        # for i in range(len(a)):
        #     if a[i]<self.distance:
        #         self.obstacle=True
        #         if self.obstacle:
        #             self.publisher_.publish(stop)
        #     else:
        #         self.publisher_.publish(forward)

    def publish_cmd(self, scan: LaserScan):

     cmd = TwistStamped()
     cmd.header.stamp = self.get_clock().now().to_msg()
     cmd.header.frame_id = "map"
 
     a = scan.ranges[0:24]+scan.ranges[337:360]

     if any(d < self.distance for d in a):  #dont go throuah all scans it stops immediatly when found less then self,distance ,instead for goijg trough all like for loop
         cmd.twist.linear.x = 0.0   # obstacle found → STOP
     else:
         cmd.twist.linear.x = 0.26  # all clear → FORWARD

     self.publisher_.publish(cmd)    # publish ONCE ✅
 
    # def stop_cmd(self):
    #     msg=TwistStamped()
    #     msg.header.stamp=self.get_clock().now().to_msg()
    #     msg.header.frame_id="map"
    #     msg.twist.linear.x=0.0
    #     self.publisher_.publish(msg)

    
    
       

def main(args=None):
    rclpy.init(args=args)
    node=LaserscanNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()


