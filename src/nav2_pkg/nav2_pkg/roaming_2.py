#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion

class Roaming_2(Node):
    def __init__(self):
        super().__init__("roaming_2")

        self.subscriber_=self.create_subscription(
            LaserScan, #msg type
            "scan", #topic name
            self.publish_scan, #callbacn fn
            10
        )

        self.publisher_=self.create_publisher(TwistStamped,"/cmd_vel",10)
        self.odom_subscriber_=self.create_subscription(Odometry,"/odom",self.get_heading,10)
        self.gap_threshold=1.0
        self.safe_gap_list=[]
        self.motion_selector="check_front_window"
        self.selected_gap=None
        self.target_angle=None
        self.heading=0.0
        self.merged=[]

        

    def get_heading(self,msg:Odometry):
        # print("odom callback started")
        x=msg.pose.pose.orientation.x
        y=msg.pose.pose.orientation.y
        z=msg.pose.pose.orientation.z
        w=msg.pose.pose.orientation.w

        _,_,yaw=euler_from_quaternion([x,y,z,w])
        self. heading=math.degrees(yaw)  # -180 to 180
        if self. heading<0:
             self. heading=self. heading+360
        # print(self.heading)


         
         
    
    def publish_scan(self,msg:LaserScan,cmd:TwistStamped):
            # print("laserscan callback started")
            self.gap_list=[]
            self.safe_gap_list=[]
            temp_list=[]
                  
            self.front_window=msg.ranges[0:7]+msg.ranges[359:352:-1]
        
            self.remaining=msg.ranges[0:360]
            # self.remaining=msg.ranges[7:353]
            for index,ray in enumerate(self.remaining):
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

            self.merged=[]
            if self.safe_gap_list[0][0][0]==0 and self.safe_gap_list[-1][-1][0]==359:
                self.merged=self.safe_gap_list[-1]+self.safe_gap_list[0]
                self.safe_gap_list.pop(0)
                self.safe_gap_list.pop(-1)
                self.safe_gap_list.append(self.merged)

            # if self.merged:
            #  print(
            #     self.merged[0][0],
            #     self.merged[len(self.merged)//2][0],
            #     self.merged[-1][0]
            #         )
            # else:
            #      print("self.merge not found")
            # print(f"safe merge :{self.merged}")



            

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
                #   print(f"gap of width {len(gap)} start:{start} end:{end} has average distance {average} ")
            maxValue=max(self.average_list)
            maxValue_index=self.average_list.index(maxValue)
            # print(f" max average value: {maxValue} at Index {maxValue_index}")
            start_safe_gap=self.safe_gap_list[maxValue_index][0][0]
            end_safe_gap=self.safe_gap_list[maxValue_index][-1][0]
            width=len(self.safe_gap_list[maxValue_index])
            safe_gap=self.safe_gap_list[maxValue_index]
            self.middle_ray_index=width//2


            #middle ray
            self.middle_ray=safe_gap[self.middle_ray_index]  
                 
            # print(f" ")
            # print(f"{self.safe_gap_list}")
            # print(f"no of gaps{len(self.safe_gap_list)}")
            # print("\n")
            # print(f"middle ray {self.middle_ray}")

            
            self.target=self. heading+self.middle_ray[0]
            # print(f"target {self.target}")

          
            if self.target>=360:
                  self.target-=360
            elif self.target<=0:
                  self.target+=360

            self.target_angle=self.target #normalized 0,360


            self.error=self.target_angle-self.heading
            if self.error>180:
                 self.error-=360
            elif self.error<-180:
                 self.error+=360

            cmd=TwistStamped()
            print("entered if else")
            
            if (self.error <0):
                      cmd.twist.angular.z=-0.4
                    #   print(self.error)
                      print(f"selected gap Start:{start_safe_gap} End:{end_safe_gap} Average:{maxValue} Width:{width} Middle value:{self.middle_ray} Target: {self.target_angle} Error : {self.error}")

            elif self.error>0:
                     cmd.twist.angular.z=+0.4
                     print(f"selected gap Start:{start_safe_gap} End:{end_safe_gap} Average:{maxValue} Width:{width} Middle value:{self.middle_ray} Target: {self.target_angle} Error : {self.error}")
            else :
                 cmd.twist.angular.z=0.0
            self.publisher_.publish(cmd)
            print(f"Error {self.error}")


            
                

                      

            # print(f"Target: {self.target_angle} Error : {self.error}")          
                                                

            
                 

            

                
                 
                














            
            
            # print(self.safe_gap_list)
            # print(maxValue)
            # print(f"heading:{self. heading}+self.middle_ray:{self.middle_ray[0]} = target_angle{self.target_angle}")
            # for gap in self.safe_gap_list:
            #     print(len(gap), gap[0][0], gap[-1][0])

            # print(f"average list {self.average_list}")



            # counter=0
            # cmd=TwistStamped()
            # linear_speed=1.0
            # angular_speed=0.2
            # stop=0.0
            # self.obstacle_threshold=1.0

            


            

            # if self.motion_selector=="check_front_window":
              
            #   for i in self.front_window:
            #         print(f"Ray {i}")

                      
            #         if (i<self.obstacle_threshold or i==self.obstacle_threshold):
            #             counter+=1
            #             if counter==2:
            #                 self.selected_gap=self.middle_ray
            #                 self.motion_selector="rotate"
            #                 break
                    
                           
            #         elif i>self.obstacle_threshold:
            #                 counter=0


            #   if counter!=2:
            #     cmd.twist.linear.x=linear_speed  
            #     cmd.twist.angular.z=stop  
            #     print("counter=0 | moving")   


            # elif self.motion_selector=="rotate":         
             
            #     cmd.twist.linear.x=stop
            #     print(f"counter value is 2 : finding gap")

            #     m=self.selected_gap[0]


            #     if (0<=m<180 or m==180):
            #         error=m

            #         if error!=0:
            #             cmd.twist.angular.z=angular_speed
            #             print(f"Error {error}")
            #             print("0<=m<180 or m==180 | error!=0 | Rotating left")
                        
            #             self.motion_selector="rotate"

            #         elif error==0:
            #             cmd.twist.angular.z=stop
            #             self.motion_selector="check_front_window"
            #             print("0<=m<180 or m==180 | error=0 | stopping")
            #             self.selected_gap=[]
            #             self.motion_selector="check_front_window"
            #             self.selected_gap=[]

            
            #     elif (180<m<=360):
            #         error=m-360
            #         print(f"Middle ray:{m} Error {error}")

            #         if error<0:
            #             cmd.twist.angular.z=-angular_speed
            #             print("180<m<=360 | error<0 | Rotating right")
            #             self.motion_selector="rotate"
            #         elif error==0:

            #             cmd.twist.angular.z=stop
            #             print("180<m<=360 | error==0 | stopping")
            #             self.selected_gap=[]
            #             self.motion_selector="check_front_window"

            # else:


                


            # 

           
                 
                
                

            


            



            
                      
                 
            



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
            



