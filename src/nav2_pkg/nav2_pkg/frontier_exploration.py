import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class frontier_exploration(Node):
    def __init__(self):
        super().__init__("frontier_exploration")
        
        self.map_subscriber_=self.create_subscription(OccupancyGrid,"/map",self.map_callback,10)

    def map_callback(self,msg:OccupancyGrid):
        self.grid_values=msg.data
        # print((f"grid values:{self.grid_values}"))
        self.width=msg.info.width
        print(f"width:{self.width}")
        self.height=msg.info.height
        print(f"height:{self.height}")
        print(len(msg.data)%2)


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



        # grid = self.two_d_grid

        # window = 10
        # center_row = 50
        # center_col = 40

        # r_start = max(0, center_row - window)
        # r_end = min(len(grid), center_row + window + 1)

        # c_start = max(0, center_col - window)
        # c_end = min(len(grid[0]), center_col + window + 1)

        # fig, ax = plt.subplots(figsize=(10, 10))

        # for r in range(r_start, r_end):
        #     for c in range(c_start, c_end):
        #         ax.text(
        #             c - c_start + 0.5,
        #             r - r_start + 0.5,
        #             str(grid[r][c]),
        #             ha="center",
        #             va="center"
        #         )

        # for x in range(c_end - c_start + 1):
        #     ax.axvline(x)

        # for y in range(r_end - r_start + 1):
        #     ax.axhline(y)

        # ax.set_xlim(0, c_end - c_start)
        # ax.set_ylim(r_end - r_start, 0)
        # ax.set_aspect("equal")

        # plt.show()


    




def main(args=None):
    rclpy.init(args=args)
    node=frontier_exploration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
