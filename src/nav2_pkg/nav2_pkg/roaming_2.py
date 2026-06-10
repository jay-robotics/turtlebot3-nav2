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
        self.motion_selector="MOVING_FORWARD"
        self.selected_gap=None
        self.target_angle=None
        self.heading=0.0
        self.merged=[]
        self.fixed_selected_middle_ray=[]

        

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


            


          
                                                

            # print(self.safe_gap_list)
            # print(maxValue)
            # print(f"heading:{self. heading}+self.middle_ray:{self.middle_ray[0]} = target_angle{self.target_angle}")
            # for gap in self.safe_gap_list:
            #     print(len(gap), gap[0][0], gap[-1][0])

            # print(f"average list {self.average_list}")



            counter=0
            cmd=TwistStamped()
            self.linear_speed=0.7
            self.obstacle_threshold=1.0
            self.stop=0.0

            

            if self.motion_selector=="MOVING_FORWARD":
            

                    print("Entered MOVING FORWARED")
                     # front window
                    self.front_window=msg.ranges[0:7]+msg.ranges[359:352:-1]
                    for i in self.front_window:
                            print("entered for loop")
                            # print(f"Ray {i}")
                            
                            if (i<self.obstacle_threshold or i==self.obstacle_threshold):
                                
                                counter+=1
                                print(f"counter inside if {counter}")
                                if counter==2:
                                    print("counter=2 | stopping")
                                    cmd.twist.linear.x=self.stop
                                    self.publisher_.publish(cmd)
                                    self.motion_selector="FIND_BEST_GAP"
                                    break
                            elif i>self.obstacle_threshold:
                                    counter=0
                                    

                    print(f"front window: {self.front_window}")

                    if counter<2:
                        print(f"counter:{counter} | moving ahead")   
                        cmd.twist.linear.x=self.linear_speed  
                        cmd.twist.angular.z=self.stop 
                        self.publisher_.publish(cmd)



            if self.motion_selector=="FIND_BEST_GAP":
                    
                    print("Entered FIND_BEST_GAP")
                
                    self.gap_list=[]
                    self.safe_gap_list=[]
                    temp_list=[]
                        
                   
                    
                    # rays for making gap list
                    self.remaining=msg.ranges[0:360]

                    #making gap list
                    for index,ray in enumerate(self.remaining):
                        if math.isinf(ray):
                            ray=3.5
                        
                        if ray>self.gap_threshold:
                            temp_list.append((index,ray))

                        elif ray<self.gap_threshold:
                            if temp_list:
                                self.gap_list.append(temp_list)
                                temp_list=[]
                    
                    if temp_list:
                        self.gap_list.append(temp_list)

                            
                    # Front Window , making safe gap list (gaps wider then 14 only)
                    self.gap_width_threshold=14
                    for index,gap in enumerate(self.gap_list):   
                        if len(gap)>=self.gap_width_threshold:
                            self.safe_gap_list.append(self.gap_list[index])
                    print(f"safe_gap_list created {len(self.safe_gap_list)}")

                    
                    self.merged=[]
                    if self.safe_gap_list[0][0][0]==0 and self.safe_gap_list[-1][-1][0]==359:
                        self.merged=self.safe_gap_list[-1]+self.safe_gap_list[0]
                        self.safe_gap_list.pop(0)
                        self.safe_gap_list.pop(-1)
                        self.safe_gap_list.append(self.merged)
                    

                    # for g in self.safe_gap_list:
                    #      print(len(g))

                    # finding average of distance of each gap in safe gap list
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


                    # finding gap with max average 
                    self.maxValue=max(self.average_list)
                    maxValue_index=self.average_list.index(self.maxValue)
                    # print(f" max average value: {maxValue} at Index {maxValue_index}")
                    self.start_safe_gap=self.safe_gap_list[maxValue_index][0][0]
                    self.end_safe_gap=self.safe_gap_list[maxValue_index][-1][0]
                    self.width=len(self.safe_gap_list[maxValue_index])
                    safe_gap=self.safe_gap_list[maxValue_index]
                    self.middle_ray_index=self.width//2

                    #middle ray
                    self.middle_ray=safe_gap[self.middle_ray_index]  
                    self.fixed_selected_middle_ray.append(self.middle_ray[0])
                    print(f"fixed selected middle ray {self.fixed_selected_middle_ray}")
                    self.target=self. heading+self.middle_ray[0]
                    # print(f"target {self.target}")
                
                    if self.target>=360:
                        self.target-=360
                    elif self.target<=0:
                        self.target+=360

                    self.target_angle=self.target #normalized 0,360
                    print("changes to moving")
                    self.motion_selector="ROTATE"


            
            if self.motion_selector=="ROTATE":
                
                print("Entered ROTATE")
                

                # normalize error between -180 to 180
                self.error=self.target_angle-self.heading
                if self.error>180:
                    self.error-=360
                elif self.error<-180:
                    self.error+=360
                print(f"error before rotation : {self.error}")

                
                cmd=TwistStamped()
                print("entered if else")
                
                self.angular_speed_slow=0.05
                self.angular_speed_fast=1.0
                self.error_speed_threshold=10
                
                
                if (self.error <0):
                          if self.error<-self.error_speed_threshold:
                                print("speed fast")
                                cmd.twist.angular.z=-self.angular_speed_fast
                                self.publisher_.publish(cmd)
                          else:
                               print("speed low")
                               cmd.twist.angular.z=-self.angular_speed_slow
                          print(f"selected gap Start:{self.start_safe_gap} End:{self.end_safe_gap} Average:{self.maxValue} Width:{self.width} Middle value:{self.middle_ray} Heading:{self.heading} Target: {self.target_angle} Error {self.error}")
                               

                elif self.error>0:
                        if self.error<=self.error_speed_threshold: 
                              print("speed low")
                              cmd.twist.angular.z=+self.angular_speed_slow
                              self.publisher_.publish(cmd)
                        else:
                         print("speed fast")
                         cmd.twist.angular.z=+self.angular_speed_fast
                         self.publisher_.publish(cmd)
                         print(f"selected gap Start:{self.start_safe_gap} End:{self.end_safe_gap} Average:{self.maxValue} Width:{self.width} Middle value:{self.middle_ray} Heading:{self.heading} Target: {self.target_angle} Error {self.error}")
                
                else :
                     cmd.twist.angular.z=0.0
                     self.publisher_.publish(cmd)
                              
                print(f"Error {self.error}")
                print("\n")

                self.error_threshold=0.3
                if -self.error_threshold<self.error<self.error_threshold:
                     cmd.twist.angular.z=0.0
                     self.publisher_.publish(cmd)
                     print("error within threshold | MOVING FORWARD")
                     self.motion_selector="MOVING_FORWARD"

            # print(f"Target: {self.target_angle} Error : {self.error}")

                

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
            



