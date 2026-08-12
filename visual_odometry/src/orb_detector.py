#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class orbdetector(Node):

    def __init__(self):
        super().__init__('orb_detector')

        self.bridge=CvBridge()

        self.orb=cv2.ORB_create()

        self.matcher=cv2.BFMatcher(cv2.NORM_HAMMING)

        self.previous_descriptors=None
        self.previous_image = None
        self.previous_keypoints = None

        self.subs=self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )


    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        keypoints, descriptors = self.orb.detectAndCompute(image, None)

        if self.previous_descriptors is None:
            self.previous_image = image.copy()
            self.previous_keypoints = keypoints
            self.previous_descriptors = descriptors
            return

        matches = self.matcher.knnMatch(self.previous_descriptors, descriptors, k=2)

        good_matches = []

        for pair in matches:

            if len(pair)<2:
                continue

            best=pair[0]
            second=pair[1]

            if best.distance<0.75*second.distance:
                good_matches.append(best)

        print(f"Raw matches: {len(matches)}")
        print(f"Good matches: {len(good_matches)}")

        match_image = cv2.drawMatches(
            self.previous_image,
            self.previous_keypoints,
            image,
            keypoints,
            good_matches,
            None
        )

        # for match in matches[:10]:
        #     print("distance", match.distance)

        # print(f"matches:{len(matches)}")

        # update ALL THREE together, in sync
        self.previous_image = image.copy()
        self.previous_keypoints = keypoints
        self.previous_descriptors = descriptors

        keypoint_image = cv2.drawKeypoints(image, keypoints, None, color=(0, 255, 0))
        cv2.imshow("orb features", keypoint_image)
        cv2.imshow("feature matches", match_image)
        print(f"Detected keypoints: {len(keypoints)}")
        print(f"Descriptor shape: {descriptors.shape}")
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node=orbdetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()