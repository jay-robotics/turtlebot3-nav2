from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy
import math
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import math
from tf2_ros import Buffer,TransformListener
import tf2_geometry_msgs


class sematic_mapping(Node):
        def __init__(self):
            super().__init__("sematic_mapping")
            self.declare_parameter("use_sim_time",True)
            print(self.get_parameter("use_sim_time").value)
            # self.image_subscriber=self.create_subscription(Image,"/camera/image_raw",self.callback,10)
            self.bridge=CvBridge()  #converts ros image type to opencv image type(numpy array) , self.bridge is an object of CvBridge class whicih contains several functions for converting images
            self.model=YOLO("yolo11x.pt") #loads trained neural network
            self.obj_list=[]
            # self.scan_subscriber_=self.create_subscription(LaserScan,"/scan",)
            self.image_sub=Subscriber(self, Image, "/camera/image_raw")
            self.scan_sub=Subscriber(self, LaserScan, "/scan")
            self.ts=ApproximateTimeSynchronizer( [self.image_sub, self.scan_sub], queue_size=10, slop=0.1)
            self.ts.registerCallback(self.callback)
            self.tf_buffer=Buffer()
            self.tf_listener=TransformListener(self.tf_buffer,self)




        def callback(self,img_msg:Image, scan_msg:LaserScan):
            self.image=img_msg
            self.ranges=scan_msg.ranges
            # print(f"0 deg:{self.ranges[0]} 90 deg:{self.ranges[90]}, 180 deg:{self.ranges[180]} 270 deg:{self.ranges[270]}")
            self.frame=self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="bgr8")   #mag=ros2 image that arrived from the camera feed  (type=senseor_msg.msg.Image),  .imgmsg_to_cv2 is one of the methods(functions) insdie CVBridge object which convert ros image to opencv image , msg=the image we want to convert, desired_encoding="bgr8" measn give output img in BGR format wiht 8 bit per color channel,after conversion it return and opencv image thaat is stored in self.frame
                # Run YOLO
            results = self.model(self.frame,conf=0.8)  #this variable stores the detected objects


            # Get the first (and only) result
            # print(results)

            for result in results:
                annotated_frame = result.plot()

                for box in result.boxes:
                    x1,y1,x2,y2=box.xyxy[0] #when detects it returns bounding box as box.xyxy=(x1,y1,x2,y2),(x1,y1)= top left corner ,(x2,y2)=botton right corner
                    centrex=int((x1+x2)/2)
                    centrey=int((y1+y2)/2)
                    cv2.circle(annotated_frame,(centrex,centrey),5,(0,0,255),-1)  #cv2.circle(image, centre, radius, color(B,G,R), thickness(2=draws only border, -1=fills the circle))
                    class_id = int(box.cls[0])
                    object_name = result.names[class_id]
                    confidence = float(box.conf[0])

                    present = False

                    for obj, _ in self.obj_list:
                        if obj == object_name:
                            present = True
                            break

                    if not present:
                        self.obj_list.append((object_name, confidence))

            print(self.obj_list)
            # #Draw bounding boxes and labels on the detected image
            height,width=annotated_frame.shape[:2]   #finding the size of image(frame),annoted_frame is a numpy array which has .shape property which give (height,width,shape)
            centre_x=width//2 # // because opencv drawing fns expect integers but / gives decimals
            centre_y=height//2
            cv2.line(annotated_frame,(centre_x,0),(centre_x,height),(0,255,0),2)  #cv2.line(image, start_point, end_point, color, thickness)
            cv2.line(annotated_frame,(0,centre_y),(width,centre_y),(0,255,0),2)

            # height, width = annotated_frame.shape[:2]

            # centre_x = width // 2
            # centre_y = height // 2

            # Long enough to reach outside the image
            length = max(width, height) * 2

            for angle in range(0,360,3):

                theta = math.radians(angle)

                # Your convention:
                # 0° = Up
                # 90° = Left
                # 180° = Down
                # 270° = Right

                end_x = int(centre_x - length * math.sin(theta))
                end_y = int(centre_y - length * math.cos(theta))

                cv2.line(
                    annotated_frame,
                    (centre_x, centre_y),
                    (end_x, end_y),
                    (225, 0, 0),
                    1
                )
                 # Position the angle text slightly before the end of the ray
                text_x = int(centre_x - (100) * math.sin(theta))
                text_y = int(centre_y - (100) * math.cos(theta))
                # print(text_x,text_y)

                # Draw the angle value
                # cv2.putText(
                #     annotated_frame,
                #     str(angle),
                #     (text_x, text_y),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     0.3,
                #     (0, 225, 0),
                #     1,
                #     cv2.LINE_AA
                # )

            ## displays the annotated image
            angle=279
            theta=math.radians(angle)  #because sin and cos requies radians
            self.ray=self.ranges[angle]
            # converting polar(lidar coordinates(distance,angle)) to cartesian(x,y,z)( for TF2)
            self.x=self.ray*math.cos(theta)
            self.y=self.ray*math.sin(theta)

            point=PointStamped() #point and header, represent a point in coordinate frame
            point.header.frame_id="base_scan"  #saying these x,y coordinates are measured in base scan frame
            point.header.stamp=self.get_clock().now().to_msg()

            point.point.x=self.x #points in current frame
            point.point.y=self.y
            point.point.z=0.0

            # converting point in base scan frame to map frame
            point_map=self.tf_buffer.transform(
                 point,
                 "map"
            )
            x_map=point_map.point.x
            y_map=point_map.point.y

            
            cv2.imshow("YOLO Detection", annotated_frame)
            cv2.waitKey(1)



        
        

        # # results=model("/home/jay/turtlebot3_ws/src/nav2_pkg/nav2_pkg/parts-80-1.avif")
        # results=model(source=1,stream=True,conf=0.5)
        # # print(results)
        # # result = results[0]   # Get the result for the first (and only) image
        # # results = model("/home/jay/Downloads/video.mp4", stream=True)

        # for result in results:
        #     # print(result)
        #     annoted_frame=result.plot()

        #     cv2.imshow("YOLO Detection",annoted_frame)

        #     for box in result.boxes:
        #         class_id = int(box.cls[0])
        #         confidence = float(box.conf[0])
        #         print(result.names[class_id], confidence)

        #     if cv2.waitKey(1)==ord('q'):
        #         break
        
def main(args=None):
     rclpy.init(args=args)
     node=sematic_mapping()
     rclpy.spin(node)
     rclpy.shutdown()

if __name__=="__main__":
     main()
     