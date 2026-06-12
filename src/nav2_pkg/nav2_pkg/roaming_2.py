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
        self.safe_gap_list=[]
        self.motion_selector="MOVING_FORWARD"
        self.selected_gap=None
        self.target_angle=None
        self.heading=0.0
        self.merged=[]
        self.fixed_selected_middle_ray=[]
        self.closest_middle_ray=[]
        
        self.declare_parameter("gap_threshold",0.7)
        self.gap_threshold=self.get_parameter("gap_threshold").value
        

        print("Paramters: window:14 gap_width:14 obstacle:1.0")
        self.declare_parameter("window",14)
        self.front_window_width=self.get_parameter("window").value

        self.declare_parameter("gap_width",14)
        self.gap_width_threshold=self.get_parameter("gap_width").value

        self.declare_parameter("obstacle",1.0)
        self.obstacle_threshold=self.get_parameter("obstacle").value

        self.declare_parameter("linear_speed",1.0)
        self.linear_speed=self.get_parameter("linear_speed").value
        self.stop=0.0
        self.angular_speed_slow=0.05
        self.angular_speed_fast=1.0
        self.error_speed_threshold=3
        self.error_threshold=0.3


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
            # print(f"frony window :{self.front_window_width}")
            # print(f"gap_width_threshold:{self.gap_width_threshold}")
            # print(f"obstacle threshold :{self.obstacle_threshold}")


            


          
                                                

            # print(self.safe_gap_list)
            # print(maxValue)
            # print(f"heading:{self. heading}+self.middle_ray:{self.middle_ray[0]} = target_angle{self.target_angle}")
            # for gap in self.safe_gap_list:
            #     print(len(gap), gap[0][0], gap[-1][0])

            # print(f"average list {self.average_list}")



            counter=0
            cmd=TwistStamped()
            # print(f"gap_width_threshold:{self.gap_width_threshold}")
            # print(f"obstacle threshold :{self.obstacle_threshold}")
            self.cmd=cmd
            


            

            if self.motion_selector=="MOVING_FORWARD":
            

                    print("Moving Forward")
                    # print(f"Front window width: {self.front_window_width}")
                     # front window
                    
                    if self.front_window_width%2==0:
                        self.rays=(self.front_window_width//2)
                        # print(f"self.rays {self.rays}")
                        self.front_window=msg.ranges[0:self.rays]+msg.ranges[359:(359-self.rays):-1]
                        # print(f"length of front window:{len(self.front_window)}")
                    elif self.front_window_width%2!=0:
                         self.rays=self.front_window_width//2
                        #  print(f"self.rays {self.rays}")
                         self.front_window=msg.ranges[0:self.rays]+msg.ranges[359: (359-(self.front_window_width-self.rays)) :-1]
                         
                    # self.front_window=msg.ranges[0:7]+msg.ranges[359:352:-1]
                    for i in self.front_window:
                            # print("entered for loop")
                            # print(f"Ray {i}")
                            
                            if (i<self.obstacle_threshold or i==self.obstacle_threshold):
                                
                                counter+=1
                                # print(f"counter inside if {counter}")
                                if counter==2:
                                    # print("counter=2 | stopping")
                                    self.cmd.twist.linear.x=self.stop
                                    self.publisher_.publish(self.cmd)
                                    
                                    self.motion_selector="FIND_BEST_GAP"
                                    break
                            elif i>self.obstacle_threshold:
                                    counter=0
                                    

                    # print(f"front window: {self.front_window}")

                    if counter<2:
                        # print(f"counter:{counter} | moving ahead")   
                        self.cmd.twist.linear.x=self.linear_speed  
                        self.cmd.twist.angular.z=self.stop 
                        self.publisher_.publish(self.cmd)



            if self.motion_selector=="FIND_BEST_GAP":
                    
                    print("Finding best gap")
                
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
                    
                    for index,gap in enumerate(self.gap_list):   
                        if len(gap)>=self.gap_width_threshold:
                            self.safe_gap_list.append(self.gap_list[index])
                    # print(f"safe_gap_list created {len(self.safe_gap_list)}")

                    
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
                    self.middle_ray_list=[]
                    for self.gap in self.safe_gap_list:
                        self.length=len(gap)
                        print(self.length)
                        self.middle_index=self.length//2
                        print(f"middle index:{self.middle_index}")
                        print(f"{self.gap}")
                        self.middleRay=self.gap[self.middle_index][0]
                        print(f"middleRay:{self.middleRay}")
                        self.middle_ray_list.append(self.middleRay)
                    
                    self.least_middle_ray=min(self.middle_ray_list)
                    

                    self.middle_ray=safe_gap[self.middle_ray_index]  
                    self.len_of_gaps=[len(i) for i in self.gap_list]
                    self.len_gaps_in_safe_gap_list={len(i) for i in self.safe_gap_list}
                    print(f"length of gap list:{(len(self.gap_list))} length of gaps in gap list:{self.len_of_gaps} length of safe_gap_list:{len(self.safe_gap_list)} len of gap in safe gap list:{self.len_gaps_in_safe_gap_list}  Length of middle gap list:{len(self.middle_ray_list)} Middle ray list:{self.middle_ray_list} selected ray:{self.least_middle_ray} min middle ray:{self.least_middle_ray}")
                    self.fixed_selected_middle_ray.append(self.middle_ray[0])
                    # print(f"fixed selected middle ray {self.fixed_selected_middle_ray}")
                    # self.target=self. heading+self.middle_ray[0]
                    self.target=self. heading+self.least_middle_ray
                    # print(f"target {self.target}")
                
                    if self.target>=360:
                        self.target-=360
                    elif self.target<=0:
                        self.target+=360

                    self.target_angle=self.target #normalized 0,360
                    # print("changes to moving")
                    self.motion_selector="ROTATE"
                    # self.motion_selector="FIND_BEST_GAP"


            
            # if self.motion_selector=="ROTATE":
                
            #     print("Rotating")
                

            #     # normalize error between -180 to 180
            #     self.error=self.target_angle-self.heading
            #     if self.error>180:
            #         self.error-=360
            #     elif self.error<-180:
            #         self.error+=360
            #     # print(f"error before rotation : {self.error}")

            #     # self.self.cmd=self.cmd
            #     # self.self.cmd=TwistStamped()
            #     # print("entered if else")
                

                
                
            #     if (self.error <0):
            #               if self.error<-self.error_speed_threshold:
            #                     # print("speed fast")
            #                     self.cmd.twist.angular.z=-self.angular_speed_fast
            #                     self.publisher_.publish(self.cmd)
            #               else:
            #                 #    print("speed low")
            #                    self.cmd.twist.angular.z=-self.angular_speed_slow
            #             #   print(f"selected gap Start:{self.start_safe_gap} End:{self.end_safe_gap} Average:{self.maxValue} Width:{self.width} Middle value:{self.middle_ray} Heading:{self.heading} Target: {self.target_angle} Error {self.error}")
                               

            #     elif self.error>0:
            #             if self.error<=self.error_speed_threshold: 
            #                 #   print("speed low")
            #                   self.cmd.twist.angular.z=+self.angular_speed_slow
            #                   self.publisher_.publish(self.cmd)
            #             else:
            #             #  print("speed fast")
            #              self.cmd.twist.angular.z=+self.angular_speed_fast
            #              self.publisher_.publish(self.cmd)
            #             #  print(f"selected gap Start:{self.start_safe_gap} End:{self.end_safe_gap} Average:{self.maxValue} Width:{self.width} Middle value:{self.middle_ray} Heading:{self.heading} Target: {self.target_angle} Error {self.error}")
                
            #     else :
            #          self.cmd.twist.angular.z=0.0
            #          self.publisher_.publish(self.cmd)
                              
            #     print(f"Error {self.error}")
            #     print("\n")

               
            #     if -self.error_threshold<self.error<self.error_threshold:
            #          self.cmd.twist.angular.z=0.0
            #          self.publisher_.publish(self.cmd)
            #          print("Target reached | Moving Forward")
            #          self.motion_selector="MOVING_FORWARD"

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



# def main(args=None):
#     rclpy.init(args=args)
#     node=Roaming_2()
#     rclpy.spin(node)
#     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Roaming_2()
    rclpy.spin(node)
    rclpy.shutdown()
 

    
        
        

    

if __name__=="__main__":
    main()
            



