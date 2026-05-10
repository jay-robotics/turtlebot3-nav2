#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator:BasicNavigator,position_x,position_y,orientation_z):
    qx,qy,qy,qw=tf_transformations.quaternion_from_euler(0.0,0.0,orientation_z)
    pose=PoseStamped()
    pose.header.frame_id='map'
    pose.header.stamp=navigator.get_clock().now().to_msg()
    pose.pose.position.x=position_x
    pose.pose.position.y=position_y
    pose.pose.position.z=0.0
    pose.pose.orientation.x=0.0
    pose.pose.orientation.y=0.0
    pose.pose.orientation.z=orientation_z

def main():
    rclpy.init()
    nav=BasicNavigator()

    # sets initial pose
    initial_pose=create_pose_stamped(nav,0.0,0.0,0.0)
    nav.setInitialPose(initial_pose)

    # wait for nav2
    nav.waitUntilNav2Active()

    # send nav2 goal
    # pi=3.14=180
    #pi/2=1.57=90
    goal_pose=PoseStamped()

    goal_pose=create_pose_stamped(nav,2.5,1.0,1.57)
    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback=nav.getFeedback()
        print(feedback)
        
    print(nav.getResult())

    # shutdown
    rclpy.shutdown()


if __name__=='__main__':
    main()