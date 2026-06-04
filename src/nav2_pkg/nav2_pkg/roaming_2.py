#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math

class Roaming_2(Node):
    def __init__(self):
        super().__init__("roaming_2")

        self.subscriber_=self.create_subscription(
            LaserScan, #msg type
            "scan", #topic name
            self.publish_scan, #callbacn fn
            10
        )
        self.gap_threshold=1.0
        self.safe_gap_list=[]

        
    
    def publish_scan(self,msg:LaserScan):
            self.gap_list=[]
            self.safe_gap_list=[]
            temp_list=[]
                  
            self.front_window=msg.ranges[0:7]+msg.ranges[359:352:-1]
        
            self.remaining=msg.ranges[7:353]
            for index,ray in enumerate(self.remaining,start=7):
                if math.isinf(ray):
                      ray=3.5
                    #   print(f"index {index}  ray_distance:{ray}")

                 
                if ray>self.gap_threshold:
                      temp_list.append((index,ray))

                elif ray<self.gap_threshold:
                     if temp_list:
                        self.gap_list.append(temp_list)
                        temp_list=[]
            
            if temp_list:
                 self.gap_list.append(temp_list)
                     
                

            
            for index,gap in enumerate(self.gap_list):   
                 if len(gap)>=14:
                      self.safe_gap_list.append(self.gap_list[index])

            # for g in self.safe_gap_list:
            #      print(len(g))

            self.total=0
            self.average_list=[]
            for gap in self.safe_gap_list:
                for ray_index,ray_distance in gap: 
                      self.total=self.total+ray_distance
                      self.length=len(gap)
                self.average=self.total/self.length
                self.average_list.append(self.average)
                self.total=0
            
            self.start_end_index=[]
            for gap,average in zip(self.safe_gap_list,self.average_list):
                  start=gap[0][0]
                  end=gap[-1][0]
                  print(f"gap of width {len(gap)} start:{start} end:{end} has average distance {average} ")
            maxValue=max(self.average_list)
            maxValue_index=self.average_list.index(maxValue)
            print(f" max average value: {maxValue} at Index {maxValue_index}")
            start_safe_gap=self.safe_gap_list[maxValue_index][0][0]
            end_safe_gap=self.safe_gap_list[maxValue_index][-1][0]
            width=len(self.safe_gap_list[maxValue_index])
            safe_gap=self.safe_gap_list[maxValue_index]
            self.middle_ray_index=width//2


            self.middle_ray=safe_gap[self.middle_ray_index]
                 
            print(f"selected gap Start:{start_safe_gap} End:{end_safe_gap} Average:{maxValue} Width:{width} Middle value:{self.middle_ray} ")
            print("\n")
            
                      
                 
            


9
                #  print(f"gap of width {len(gap)} has average distance {average} ")
            

            # print(self.safe_gap_list)   #ray_index,ray_distance

                 

            # print(f"{len(self.safe_gap_list)}, {len(self.average_list)}")

            
            
                  
                      

            
                         
            


            # print(len(self.remaining))   #345
            # for i in self.gap_list:
            #      print(i,"\n")
            # print(self.gap_list)
            # print(len(self.gap_list))
            # print(f"gap list length {len(self.gap_list)} safe gap list length {len(self.safe_gap_list)}")
            
            # print(len(self.remaining)+len(self.front_window))
            

            # print(len(self.front_window))  #14
            # print(self.front_window)
            # print(f" first ray is {msg.ranges[0]} Last Ray is {msg.ranges[-1]}")
            
            # print(self.scans)



def main(args=None):
    rclpy.init(args=args)
    node=Roaming_2()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
            



