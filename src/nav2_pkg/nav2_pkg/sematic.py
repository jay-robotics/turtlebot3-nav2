from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy


class sematic_mapping(Node):
        def __init__(self):
            super().__init__("sematic_mapping")

            self.image_subscriber=self.create_subscription(Image,"/camera/image_raw",self.callback,10)
            self.bridge=CvBridge()  #converts ros image type to opencv image type(numpy array) , self.bridge is an object of CvBridge class whicih contains several functions for converting images
            self.model=YOLO("yolo11x.pt") #loads trained neural network
            self.obj_list=[]




        def callback(self,msg:Image):
            self.image=msg
            self.frame=self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")   #mag=ros2 image that arrived from the camera feed  (type=senseor_msg.msg.Image),  .imgmsg_to_cv2 is one of the methods(functions) insdie CVBridge object which convert ros image to opencv image , msg=the image we want to convert, desired_encoding="bgr8" measn give output img in BGR format wiht 8 bit per color channel,after conversion it return and opencv image thaat is stored in self.frame
                # Run YOLO
            results = self.model(self.frame,conf=0.8)  #this variable stores the detected objects

            # Get the first (and only) result
            # print(results)

            for result in results:
                for box in result.boxes:
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
            annotated_frame = result.plot()

            ## displays the annotated image
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
     