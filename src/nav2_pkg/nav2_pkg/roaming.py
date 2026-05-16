#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math


class Roaming(Node):
    def __init__(self):
        super().__init__("roaming")
        
        self.subscriber_=self.create_subscription(LaserScan,"scan",self.publish_scan,10)
        self.publisher_=self.create_publisher(TwistStamped,"/cmd_vel",10)
        self.declare_parameter("distance",0.7)
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
        self.windows.append(self.scan[138:184])  #46    total 8 window , 0 to 7
        self.windows.append(self.scan[184:230])
        self.windows.append(self.scan[230:276])
        self.windows.append(self.scan[276:322])
        self.windows.append(self.scan[322:337]) #15
 
        # print(len(self.windows))

        # for index,value in enumerate(self.scan):
        #     print(f"index:{index} Value:{value}")
        # self.flag=True


        # if any(d<self.threshold for d in self.windows[0]):
        #     cmd.twist.linear.x=0.0
        # else:
        #     cmd.twist.linear.x=0.25

        # self.publisher_.publish(cmd)



        # for windowIndex,window in enumerate(self.windows):
        #     for index,rays in enumerate(window):
        #         print(f"window index:{windowIndex} index:{index,rays}")


        self.deviations=[]   #deviations list
        self.averageList=[]   #average dsistance list
        for window in self.windows[1:8]:  #get first window array

            valid_window=[]

            for ray in window:
                if math.isinf(ray) or math.isnan(ray):
                    ray=3.5
                    valid_window.append(ray)
                else:
                    valid_window.append(ray)

            if len(valid_window)==0:
                continue

                                                                                                        # for index,ray in enumerate(window):
                                                                                                                        #     print(f" rays {index}:{ray}")

            self.average=( sum(valid_window) / (len(valid_window)) )
            self.averageList.append(self.average)
            self.tempList=[]
            

            for ray in valid_window: 
               self.deviation=abs(ray - self.average)  #1
               self.tempList.append(self.deviation) #2
            #    print(f"list len {len(self.tempList)}")
            self.deviations.append(self.tempList) #3
       
        
        self.sum_of_deviatons=[]  #
        for deviationsList in self.deviations:
                
                self.add=sum(deviationsList)
                self.sum_of_deviatons.append(self.add)
        # print(self.sum)
        # print(f" average list {self.averageList} Max Value:{max(self.averageList)}")
        # print(f"sum of deviations {self.sum_of_deviatons} Min Value:{min(self.sum_of_deviatons)}")

        max_averageValue=max(self.averageList)
        max_index=self.averageList.index(max_averageValue)
        
        middle_angle_list=[]
        for index,w in enumerate(self.windows):
            window_start = (337 + (index * 46)) % 360
            middle_ray = len(w) // 2
            middle_angle = (window_start + middle_ray) % 360
            middle_angle_list.append(middle_angle)
            # print(f"Window {index} middle angle: {middle_angle}")
            
        # print(middle_angle_list)
        # print(f"Window {max_index} has Max Value:{max_averageValue} with middle rays:{middle_angle_list[max_index]}")
        m=middle_angle_list[max_index+1]
        print(f"Window {max_index} has Max Value:{max_averageValue} with middle rays:{m}")

        if m < 10 or m > 350:
            cmd.twist.linear.x = 0.25
            cmd.twist.angular.z = 0.0
        elif 0 < m <= 180:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.5   # rotate left
        else:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = -0.5  # rotate right

        self.publisher_.publish(cmd)



       
    

        






        



                                               
            




        
      


        






def main(args=None):
    rclpy.init(args=args)
    node=Roaming()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()

