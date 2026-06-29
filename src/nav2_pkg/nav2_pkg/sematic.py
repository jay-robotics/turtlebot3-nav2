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
            self.bridge=CvBridge()
            self.model=YOLO("yolo11x.pt")




        def callback(self,msg:Image):
            self.image=msg
            self.frame=self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
                # Run YOLO
            results = self.model(self.frame)

            # Get the first (and only) result
            result = results[0]

            # Draw bounding boxes and labels
            annotated_frame = result.plot()

            # Show the annotated image
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
        # # for box in result.boxes:
        # #     class_id = int(box.cls[0])   # Convert tensor to an integer
        # #     class_name = result.names[class_id]   # Look up the class name
        # #     confidence = float(box.conf[0])   # Confidence score

        # #     print(f"Object: {class_name}, Confidence: {confidence:.2f}")

def main(args=None):
     rclpy.init(args=args)
     node=sematic_mapping()
     rclpy.spin(node)
     rclpy.shutdown()

if __name__=="__main__":
     main()
     