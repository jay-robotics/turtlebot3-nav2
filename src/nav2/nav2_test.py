#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator:BasicNavigator,position_x,position_y,orientation_z):
    qx,qy,qz,qw=tf_transformations.quaternion_from_euler(0.0,0.0,orientation_z)
    pose=PoseStamped()
    pose.header.frame_id='map'
    pose.header.stamp=navigator.get_clock().now().to_msg()
    pose.pose.position.x=position_x
    pose.pose.position.y=position_y
    pose.pose.position.z=0.0
    pose.pose.orientation.x=qx
    pose.pose.orientation.y=qy
    pose.pose.orientation.z=qz
    pose.pose.orientation.w=qw
    return pose

def main():
    rclpy.init()
    nav=BasicNavigator()

    # sets initial pose
    initial_pose=create_pose_stamped(nav,0.0,0.0,0.0)
    nav.setInitialPose(initial_pose)

    # wait for nav2
    nav.waitUntilNav2Active()

   # nav2 poses
    goal_pose1=create_pose_stamped(nav,3.5,1.0,1.57)
    goal_pose2=create_pose_stamped(nav,2.0,2.5,3.14)
    goal_pose3=create_pose_stamped(nav,0.5,1.0,-1.57)

    # #go to one pose
    # nav.goToPose(goal_pose1)
    # while not nav.isTaskComplete():
    #     feedback=nav.getFeedback()
    #     print(feedback)

    #follow waypoints
    waypoints=[goal_pose1,goal_pose2,goal_pose3]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        print(nav.getFeedback())
        
    print(nav.getResult())

    # shutdown
    rclpy.shutdown()


if __name__=='__main__':
    main()
