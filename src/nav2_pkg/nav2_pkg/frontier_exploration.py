import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from collections import deque
from tf2_ros import Buffer,TransformListener

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


    def map_callback(self,msg:OccupancyGrid):
        self.grid_values=msg.data
        # print((f"grid values:{self.grid_values}"))
        self.width=msg.info.width
        print(f"width:{self.width}")
        self.height=msg.info.height
        print(f"height:{self.height}")
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
            robot_col=int( (robot_x-self.origin_x)/self.resolution)
            robot_row=int((robot_y-self.origin_y)/self.resolution)
            print(robot_row,robot_col)
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

        print(f"lenght of self.two_f_grid:{len(self.two_d_grid)}")
        print(f"2d grid:{self.two_d_grid}")

        self.frontier_cell_list=[]


        width=self.width
        height=self.height
        print(f"width,height;{(width,height)}")

       
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
                                
        print(*self.frontier_cell_list,sep="\n")

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
        print(f"main clustor: {self.cluster_main}")

                


                            


                
                        
        # print("\n".join(selected_list))
            




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
