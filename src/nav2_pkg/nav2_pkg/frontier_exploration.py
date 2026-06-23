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

MIN_FRONTIER_DISTANCE_CELLS=10
MIN_FRONTIER_CLUSTER_SIZE=7

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
        self.visited_cell=[]

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
        self.width=msg.info.width
        self.height=msg.info.height
        # print(f"height:{self.height} width:{self.width}")
        print(len(msg.data)%2)
        self.resolution=msg.info.resolution
        self.origin_x=msg.info.origin.position.x
        self.origin_y=msg.info.origin.position.y
        # print(f"resolution:{self.resolution} origin_x:{self.origin_x} origin_y:{self.origin_y}")

        pose=self.get_robot_pose()  #get robots current position in x,y form

        if pose is not None:
            robot_x,robot_y,robot_or_x,robot_or_y,robot_or_z,robot_or_w=pose
            self.robot_col=int( (robot_x-self.origin_x)/self.resolution)
            self.robot_row=int((robot_y-self.origin_y)/self.resolution)
            # print(f"robot (x,y):({robot_x},{robot_y}) robot (r,c):({self.robot_row},{self.robot_col})")
        elif pose is None:
            print(f"pose returned None")
            return False
        
        #convert grid value to 2d
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

        #frontier cell list (4 sides)
        self.frontier_cell_list=[]

        width=self.width
        height=self.height
       
        self.frontier_cell_list=[]

        width=self.width
        height=self.height
       
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

                if r-1>=0 and c-1>=0:
                    upper_left=self.two_d_grid[r-1][c-1]
                else:
                    upper_left=False

                if r-1>=0 and c+1<width:
                    upper_right=self.two_d_grid[r-1][c+1]
                else:
                    upper_right=False

                if r+1<height and c-1>=0:
                    lower_left=self.two_d_grid[r+1][c-1]
                else:
                    lower_left=False

                if r+1<height and c+1<width:
                    lower_right=self.two_d_grid[r+1][c+1]
                else:
                    lower_right=False
                
                if self.two_d_grid[r][c]==0:
                        
                    if (upper==-1) or (lower==-1) or (right==-1) or (left==-1) or (upper_left==-1) or (upper_right==-1) or (lower_left==-1) or (lower_right==-1):
                        self.frontier_cell_list.append((r,c))
                                
        # print("frontier cell list",*self.frontier_cell_list,sep="\n")

        # making frontier clusters
        for cell in self.frontier_cell_list:
         if cell not in self.visited:   
            
            self.queue.append(cell)     
            self.visited[cell]=True
            
            while(self.queue):
               self.current=self.queue[0]
               if self.current not in self.cluster:
                  self.cluster.append(self.current)
               self.queue.popleft()
               
               self.r,self.c=self.current
               self.lower=(self.r+1,self.c)
               self.left=(self.r,self.c-1)
               self.upper=(self.r-1,self.c)
               self.right=(self.r,self.c+1)
               self.upper_left=(self.r-1,self.c-1)
               self.upper_right=(self.r-1,self.c+1)
               self.lower_left=(self.r+1,self.c-1)
               self.lower_right=(self.r+1,self.c+1)
               
               if self.lower in self.frontier_cell_list and self.lower not in self.visited:
                  self.queue.append(self.lower)
                  self.visited[self.lower]=True
                  self.cluster.append(self.lower)
                  
               if self.upper in self.frontier_cell_list and self.upper not in self.visited :
                  self.queue.append(self.upper)
                  self.visited[self.upper]=True
                  self.cluster.append(self.upper)
                  
               if self.left in self.frontier_cell_list and self.left not in self.visited:
                  self.queue.append(self.left)
                  self.visited[self.left]=True
                  self.cluster.append(self.left)
                  
               if self.right in self.frontier_cell_list and self.right not in self.visited:
                  self.queue.append(self.right)
                  self.visited[self.right]=True
                  self.cluster.append(self.right)

               if self.upper_left in self.frontier_cell_list and self.upper_left not in self.visited:
                  self.queue.append(self.upper_left)
                  self.visited[self.upper_left]=True
                  self.cluster.append(self.upper_left)

               if self.upper_right in self.frontier_cell_list and self.upper_right not in self.visited:
                  self.queue.append(self.upper_right)
                  self.visited[self.upper_right]=True
                  self.cluster.append(self.upper_right)

               if self.lower_left in self.frontier_cell_list and self.lower_left not in self.visited:
                  self.queue.append(self.lower_left)
                  self.visited[self.lower_left]=True
                  self.cluster.append(self.lower_left)

               if self.lower_right in self.frontier_cell_list and self.lower_right not in self.visited:
                  self.queue.append(self.lower_right)
                  self.visited[self.lower_right]=True
                  self.cluster.append(self.lower_right)
                  
            self.cluster_main.append(self.cluster)
            self.cluster=[]
            self.cluster_len=[]
                  
               

        self.cluster_len=[]
        for index,c in enumerate(self.cluster_main):
            leng=len(c)
            self.cluster_len.append((index,leng))
        
        sorted_cluster=sorted(self.cluster_len,key=lambda x:x[1],reverse=True)
        # print(f"sorted cluster:{sorted_cluster}")
        minimum=min(sorted_cluster,key=lambda x:x[1])
        maximum=max(sorted_cluster,key=lambda x:x[1])
        print(f"len of sorted cluster list:{len(sorted_cluster)} min:{minimum}, max:{maximum}")
        self.selected_large_cluster=self.cluster_main[maximum[0]]
        # print(f"largest selected cluster:{self.selected_large_cluster}")
        


        print("\n========== FRONTIER DEBUG ==========")
        print("Total clusters:", len(self.cluster_main))

        if i in self.cluster_main==0:
            print("equal to zero")

        if not self.cluster_main:
            self.get_logger().info("No frontier clusters found.")
            return False

        if len(self.cluster_main) > 0:
            sizes = [len(c) for c in self.cluster_main]

            print(f"Largest cluster size:{max(sizes)} Smallest cluster size:{min(sizes)}")
            self.sizes_sorted = sorted(sizes, reverse=True)
            print(f"len of cluster:{len(self.cluster_main)} len of sorted:{len(self.sizes_sorted)}")

        #     print(f"Top 20 cluster sizes:{self.sizes_sorted[:20]}")
        #     print("\nFirst 20 clusters:")
        #     for i, cluster in enumerate(self.cluster_main[:20]):
        #         print(
        #             f"cluster {i} "
        #             f"size={len(cluster)} "
        #             f"sample={cluster[:5]}"
        # )


            # print(f"OUT OF WHILE LOOP self.cluster {self.cluster} visited {self.visited} queue:{self.queue}")
        # print(f"main cluster: {self.cluster_main}")
        # in r,c



        # find averages to represent them for finding distance

        self.cluster_avg=[]  #index,(r,c)
        for index,cluster in enumerate(self.cluster_main):
            r_avg=0
            c_avg=0
            for r,c in cluster:
                r_avg+=r/(len(cluster))
                c_avg+=c/(len(cluster))
            self.cluster_avg.append((index,(r_avg,c_avg)))
        


        #finding distance of cluster from robot

        self.distance_list=[]     #(index,distance)
        for index,i in self.cluster_avg:
            row,col=i
            self.distance = math.sqrt((self.robot_col - col)**2 + (self.robot_row - row)**2)
            self.distance_list.append((index,self.distance))
        print(f"len cluster list:{len(self.cluster_main)} len cluster avg list;{len(self.cluster_avg)} len distance_list:{len(self.distance_list)} ")
        # print(self.distance_list)



        #only keeping distances greater then threshold

        self.far_distance_list=[]    #(index,distance)
        for index,d in self.distance_list:
            if d>MIN_FRONTIER_DISTANCE_CELLS:
                self.far_distance_list.append((index,d))
        print(f"clusters in far_distance_list:{len(self.far_distance_list)}")
        # print(f"far distance list{self.far_distance_list}")

        if not self.far_distance_list:
            self.get_logger().info("No frontier cluster far enough from the robot.")
            return False
        
        self.cluster_scores=[]   # (index, score)
        for index,d in self.far_distance_list:
            size=len(self.cluster_main[index])
            score=size/d if d>0 else size
            self.cluster_scores.append((index,score))

        best_index,best_score=max(self.cluster_scores,key=lambda x:x[1])
        print(f"best scored cluster -> index:{best_index} score:{best_score} size:{len(self.cluster_main[best_index])}")
        

        


        # only keeping goals passing threshold and minimum cluster size

        self.goal_candidates=[]  # (index,distance)
        for index,distance in self.far_distance_list:
            if len(self.cluster_main[index])>=MIN_FRONTIER_CLUSTER_SIZE:
                self.goal_candidates.append((index,distance))
        print(f"goal candidates len:{len(self.goal_candidates)}")
        for idx,dist in self.goal_candidates:
            print(f"size:{len(self.cluster_main[idx])}, Distance:{dist}")

        if not self.goal_candidates:
            print(f"no goal candidates:{self.far_distance_list}")
            print(self.sizes_sorted)
            self.goal_candidates=self.far_distance_list




        self.index1, self.distance = min(self.goal_candidates, key=lambda x: x[1])

        self.min_custor=self.cluster_main[self.index1]
        self.min_avg=self.cluster_avg[self.index1]
        self.max=max(self.goal_candidates, key=lambda x: x[1])
        self.max_index=self.distance_list.index(self.max)
        self.max_custor=self.cluster_main[self.index1]
        print(
                f"min distance:{self.distance} index:{self.index1} cluster:{self.min_custor} length:{len(self.min_custor)}\n"
                f"max distance:{self.max[1]} index:{self.max_index} cluster:{self.max_custor} length:{len(self.max_custor)}\n"
            )
        # print(*self.far_distance_list,sep="\n")


        idx=best_index
        idx=maximum[0]
        _,(avg_row,avg_col)=self.cluster_avg[idx]
        self.selected_cell=max(
            self.cluster_main[idx],
            key=lambda cell: math.sqrt((cell[0]-avg_row)**2 + (cell[1]-avg_col)**2)
        )
        # print(f"\nSelected frontier: index:{maximum[0]} size:{len(self.cluster_main[maximum[0]])} distance:{self.distance_list[maximum[0]]} cluster:{self.cluster_main[maximum[0]]}")
        print(f"\nSelected frontier: index:{best_index} size:{len(self.cluster_main[best_index])} distance:{self.distance_list[best_index]} cluster:{self.cluster_main[best_index]}")
        print(f"robott pose:{self.robot_row,self.robot_col} selected cell:{self.selected_cell} ")
        self.r,self.c=self.selected_cell
        self.visited_cell.append((self.r,self.c))

        # convert (r,c) in x,y
        self.des_x=((self.origin_x)+(self.c*self.resolution))+(self.resolution/2)
        self.des_y=((self.origin_y)+(self.r*self.resolution))+(self.resolution/2)
        # print(f"cell in x,y after conversion:{(self.des_x,self.des_y)}")

        self.goal=PoseStamped()
        self.goal.header.frame_id="map"
        self.goal.header.stamp=self.get_clock().now().to_msg()
        self.goal.pose.position.x=self.des_x
        self.goal.pose.position.y=self.des_y
        self.goal.pose.orientation.x=robot_or_x
        self.goal.pose.orientation.y=robot_or_y
        self.goal.pose.orientation.z=robot_or_z
        self.goal.pose.orientation.w=robot_or_w
        # self.goal.pose.orientation.w=1.0

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("NavigateToPose action server is not available yet.")
            return False
        # print(f"\nSelected frontier: index:{self.index1} size:{len(self.cluster_main[self.index1])} distance:{self.distance}")
        print(f"\nSelected frontier: index:{maximum[0]} size:{len(self.cluster_main[maximum[0]])} distance:{self.distance_list[maximum[0]]} cluster:{self.cluster_main[maximum[0]]}")
        # print("index =", self.index1)
        # print("size =", len(self.cluster_main[self.index1]))
        # print("distance =", self.distance)
        # print("cluster =", self.cluster_main[self.index1])
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

            rx=transform.transform.rotation.x
            ry=transform.transform.rotation.y
            rz=transform.transform.rotation.z
            rw=transform.transform.rotation.w
            return x,y,rx,ry,rz,rw
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
