import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from collections import deque
from tf2_ros import Buffer,TransformListener
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

MIN_FRONTIER_DISTANCE_CELLS=20
MIN_FRONTIER_CLUSTER_SIZE=3

class frontier_exploration(Node):
    def __init__(self):
        super().__init__("frontier_exploration")
        
        self.map_subscriber_=self.create_subscription(OccupancyGrid,"/map",self.map_callback,10)
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.queue=deque()
        self.visited={}
        self.current=[]
        self.cluster=[]
        self.cluster_main=[]
        self.state="finding frontier"
        self.visited_closest_cell=[]

        # Blocks map processing while choosing or driving to a goal.
        self.is_navigating=False

        self.nav_client=ActionClient(self,NavigateToPose,"navigate_to_pose")
        self.goal_handle=None

    def map_callback(self,msg:OccupancyGrid):

        if self.is_navigating:
            return

        self.is_navigating=True
        goal_sent=False

        try:
            goal_sent=self.process_map_and_send_goal(msg)
        except Exception as e:
            self.get_logger().warn(f"Could not send frontier goal: {e}")
        finally:
            if not goal_sent:
                self.is_navigating=False


    def process_map_and_send_goal(self,msg:OccupancyGrid):
        self.grid_values=msg.data
        self.queue=deque()
        self.visited={}
        self.cluster=[]
        self.cluster_main=[]
        # print((f"grid values:{self.grid_values}"))
        self.width=msg.info.width
        # print(f"width:{self.width}")
        self.height=msg.info.height
        # print(f"height:{self.height}")
        print(len(msg.data)%2)
        self.resolution=msg.info.resolution
        self.origin_x=msg.info.origin.position.x
        self.origin_y=msg.info.origin.position.y
        print(f"resolution:{self.resolution} origin_x:{self.origin_x} origin_y:{self.origin_y}")

        pose=self.get_robot_pose()

        if pose is not None:
            robot_x,robot_y=pose
            print(f"robot x={robot_x}")
            print(f"robot y={robot_y}")
            self.robot_col=int( (robot_x-self.origin_x)/self.resolution)
            self.robot_row=int((robot_y-self.origin_y)/self.resolution)
            # print(self.robot_row,self.robot_col)
        elif pose is None:
            print(f"pose returned None")
            return False
        self.two_d_grid=[]
        self.temp=[]
        for i in self.grid_values:
            if len(self.temp)<self.width:
                self.temp.append(i)
            elif len(self.temp)>=self.width:
                self.two_d_grid.append(self.temp)
                self.temp=[]
                self.temp.append(i)
        if self.temp:
                self.two_d_grid.append(self.temp)

        # print(f"lenght of self.two_f_grid:{len(self.two_d_grid)}")
        # print(f"2d grid:{self.two_d_grid}")

        self.frontier_cell_list=[]


        width=self.width
        height=self.height
        # print(f"width,height;{(width,height)}")

       
        for r in range(0,height):
            for c in range(0,width):

                if r-1>=0:
                    upper=self.two_d_grid[r-1][c]
                else:
                    upper=False

                if r+1<height:
                    lower=self.two_d_grid[r+1][c]
                else:
                    lower=False

                if c-1>=0:
                    left=self.two_d_grid[r][c-1]
                else:
                    left=False

                if c+1<width:
                    right=self.two_d_grid[r][c+1]
                else: 
                    right=False
                
                if self.two_d_grid[r][c]==0:
                        
                    if (upper==-1) or (lower==-1) or (right==-1) or (left==-1):
                        self.frontier_cell_list.append((r,c))
                                
        # print(*self.frontier_cell_list,sep="\n")

        for cell in self.frontier_cell_list:
         if cell not in self.visited:   
            
            # print(f"cell:{cell}")
            self.queue.append(cell)     
            self.visited[cell]=True
            # print(f" before while satrt:{cell} queue:{self.queue} visited {self.visited}")
            
         

            while(self.queue):
            #    print("enterd while")
               self.current=self.queue[0]
               if self.current not in self.cluster:
                  self.cluster.append(self.current)
               self.queue.popleft()
            #    print(f"queue:{self.queue} current:{self.current} visited {self.visited}")
               
               

               self.r,self.c=self.current
               self.lower=(self.r+1,self.c)
               self.left=(self.r,self.c-1)
               self.upper=(self.r-1,self.c)
               self.right=(self.r,self.c+1)
               
               
               if self.lower in self.frontier_cell_list and self.lower not in self.visited:
                #   print(f"self.lower {self.lower}")
                  self.queue.append(self.lower)
                  self.visited[self.lower]=True
                  self.cluster.append(self.lower)
                #   print(f"lower queue {self.queue} visited:{self.visited} cluster:{self.cluster}")
                  
               
               if self.upper in self.frontier_cell_list and self.upper not in self.visited :
                #   print(f"fself.upper {self.upper}")
                  self.queue.append(self.upper)
                  self.visited[self.upper]=True
                  self.cluster.append(self.upper)
                #   print(f"upper queue {self.queue} visited:{self.visited} cluster:{self.cluster}")
                  
               
               if self.left in self.frontier_cell_list and self.left not in self.visited:
                #   print(f"self.left {self.left}")
                  self.queue.append(self.left)
                  self.visited[self.left]=True
                  self.cluster.append(self.left)
                #   print(f"left queue {self.queue} visited:{self.visited} cluster:{self.cluster}")
                  
               
               if self.right in self.frontier_cell_list and self.right not in self.visited:
                #   print(f"self.right {self.right}")
                  self.queue.append(self.right)
                  self.visited[self.right]=True
                  self.cluster.append(self.right)
                #   print(f"right queue {self.queue} visited:{self.visited} cluster:{self.cluster}")
                  
               
            self.cluster_main.append(self.cluster)
            self.cluster=[]
            # print(f"OUT OF WHILE LOOP self.cluster {self.cluster} visited {self.visited} queue:{self.queue}")
        # print(f"main clustor: {self.cluster_main}")
        # in r,c

        if not self.cluster_main:
            self.get_logger().info("No frontier clusters found.")
            return False

        # averages
        self.cluster_avg=[]
        for index,cluster in enumerate(self.cluster_main):
            r_avg=0
            c_avg=0
            for r,c in cluster:
                r_avg+=r/(len(cluster))
                c_avg+=c/(len(cluster))
            self.cluster_avg.append((index,(r_avg,c_avg)))

        self.distance_list=[]
        for index,i in self.cluster_avg:
            row,col=i
            self.distance = math.sqrt((self.robot_col - col)**2 + (self.robot_row - row)**2)
            self.distance_list.append((index,self.distance))
        print(f"len cluster list;{len(self.cluster_main)} len cluster avg list;{len(self.cluster_avg)} len distance_list:{len(self.distance_list)} ")
        # print(self.distance_list)
        # print(self.distance_list)
        self.far_distance_list=[]
        self.index=[]
        for index,i in self.distance_list:
            # print(i)
            if i>MIN_FRONTIER_DISTANCE_CELLS:
                self.far_distance_list.append((index,i))

        # print(f"far distance list{self.far_distance_list}")
        if not self.far_distance_list:
            self.get_logger().info("No frontier cluster far enough from the robot.")
            return False

        self.goal_candidates=[
            (index,distance)
            for index,distance in self.far_distance_list
            if len(self.cluster_main[index])>=MIN_FRONTIER_CLUSTER_SIZE
        ]
        if not self.goal_candidates:
            self.goal_candidates=self.far_distance_list

        self.index1, self.distance = min(self.goal_candidates, key=lambda x: x[1])
        # self.min=min(self.far_distance_list)
        # self.min_index=self.distance_list.index(self.min)
        self.min_custor=self.cluster_main[self.index1]
        self.min_avg=self.cluster_avg[self.index1]
        self.max=max(self.distance_list)
        self.max_index=self.distance_list.index(self.max)
        self.max_custor=self.cluster_main[self.index1]
        print(
                f"min distance:{self.distance}\n"
                f"index:{self.index1}\n"
                f"main_clustor:{self.min_custor}\n"
                f"len min_clustor:{len(self.min_custor)}\n"
                f"in avglist:{self.min_avg}\n"
                f"max:{self.max} index:{self.max_index}\n"
                # f"distancelist:{self.distance_list}\n"
                f"far_distance list:{len(self.far_distance_list)}"
                f"first far"
            )
        # print(*self.far_distance_list,sep="\n")



        _,(avg_row,avg_col)=self.min_avg
        self.selected_cell=min(
            self.min_custor,
            key=lambda cell: math.sqrt((cell[0]-avg_row)**2 + (cell[1]-avg_col)**2)
        )
        print(f"robott pose:{self.robot_row,self.robot_col} selected cell:{self.selected_cell}")
        self.r,self.c=self.selected_cell

        # convert (r,c) in x,y
        self.des_x=((self.origin_x)+(self.c*self.resolution))+(self.resolution/2)
        self.des_y=((self.origin_y)+(self.r*self.resolution))+(self.resolution/2)
        # print(f"cell in x,y after conversion:{(self.des_x,self.des_y)}")

        self.goal=PoseStamped()
        self.goal.header.frame_id="map"
        self.goal.header.stamp=self.get_clock().now().to_msg()
        self.goal.pose.position.x=self.des_x
        self.goal.pose.position.y=self.des_y
        self.goal.pose.orientation.w=1.0

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("NavigateToPose action server is not available yet.")
            return False

        goal_msg=NavigateToPose.Goal()
        goal_msg.pose=self.goal

        self.get_logger().info(f"Sending frontier goal: {self.des_x:.3f}, {self.des_y:.3f}")
        send_goal_future=self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f"Sent frontier goal: {self.des_x:.3f}, {self.des_y:.3f}")
        return True


    def goal_response_callback(self,future):
        try:
            self.goal_handle=future.result()
        except Exception as e:
            self.get_logger().warn(f"Goal request failed: {e}")
            self.is_navigating=False
            return

        if not self.goal_handle.accepted:
            self.get_logger().warn("Frontier goal was rejected.")
            self.is_navigating=False
            return

        self.get_logger().info("Frontier goal accepted.")
        result_future=self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)


    def goal_result_callback(self,future):
        try:
            result=future.result()
        except Exception as e:
            self.get_logger().warn(f"Goal result failed: {e}")
            self.is_navigating=False
            return

        if result.status==GoalStatus.STATUS_SUCCEEDED:
            print("result:TaskResult.SUCCEEDED")
        else:
            print(f"result status:{result.status}")

        self.is_navigating=False



    def get_robot_pose(self):
        try:
            transform=self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            x=transform.transform.translation.x
            y=transform.transform.translation.y
            return x,y
        except Exception as e:
            self.get_logger().warn(str(e))
            return None



def main(args=None):
    rclpy.init(args=args)
    node=frontier_exploration()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()
