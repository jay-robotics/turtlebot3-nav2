# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TwistStamped
# import time

# class cmd_vel(Node):
#     def __init__(self):
#         super().__init__("commmandPublisher")
#         self.publisher_=self.create_publisher(TwistStamped,"/cmd_vel",10)
#         self.timer_=self.create_timer(1.0,self.publise_cmd)
#         self.declare_parameter("command",0.1)
#         self.speed_=self.get_parameter("command").value
     
    
#     def publise_cmd(self):
#         msg=TwistStamped()
#         msg.header.frame_id="map"
#         msg.twist.linear.x=self.speed_
#         self.publisher_.publish(msg)

#     def stop_robot(self):
#         stopmsg=TwistStamped()
#         stopmsg.header.frame_id="map"
#         stopmsg.twist.linear.x=0.0
#         self.publisher_.publish(stopmsg)

       



# def main(args=None):
#     rclpy.init(args=args)
#     node=cmd_vel()
#     try:
#      rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.stop_robot()
#         time.sleep(0.2)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
        

# if __name__=="__main__":
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__("command_publisher")
        self.publisher_ = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            10
        )
        self.declare_parameter("command", 0.1)
        self.timer_ = self.create_timer(
            1.0,
            self.publish_cmd
        )

    def publish_cmd(self):
        speed = self.get_parameter("command").value
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = speed
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Publishing speed: {speed}")

    def stop_robot(self):
     self.timer_.cancel()

     self.get_logger().info("Stopping robot...")

     stop_msg = TwistStamped()
     stop_msg.header.frame_id = "map"
     stop_msg.twist.linear.x = 0.0
     stop_msg.twist.angular.z = 0.0

     for _ in range(30):  # ✅ increased from 10 to 30
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(stop_msg)
        time.sleep(0.05)  # ✅ faster publishing = 30 msgs over 1.5 seconds

     self.get_logger().info("Robot stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()  # timer cancelled inside, then 10 stop messages
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()