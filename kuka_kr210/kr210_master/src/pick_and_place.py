#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import GripperCommandActionGoal

def perform_trajectory():

    rospy.init_node('kuka_trajectory_publisher')  

    controller_name = '/arm_controller/command'
    gripper_controller_name = '/gripper/gripper_cmd/goal'

    trajectory_publisher = rospy.Publisher(controller_name, JointTrajectory, queue_size=10)  
    gripper_publisher = rospy.Publisher(gripper_controller_name, GripperCommandActionGoal, queue_size=10)  

    kuka_joints = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']  # Name of joints

    waypoints = [
        [-1.57, 0.61, -0.349066, 0, 1.309, -1.5708],
        [0, 0, 0, 0, 0, 0.75],
        [-1.57, 0.61, -0.349066, 0, 1.309, -1.5708],
        [0, 0, 0, 0, 0, 0.75]
    ]

    gripper_positions = [0.18, 0.0, 0.18, 0.0]  

    rospy.loginfo("Starting trajectory movement")

    for idx, point in enumerate(waypoints):      
        rospy.loginfo("Moving to waypoint {}".format(idx+1))  
        trajectory_msg = JointTrajectory()  
        trajectory_msg.joint_names = kuka_joints
        trajectory_msg.points.append(JointTrajectoryPoint())
        trajectory_msg.points[0].positions = point
        trajectory_msg.points[0].velocities = [0.0 for i in kuka_joints]
        trajectory_msg.points[0].accelerations = [0.0 for i in kuka_joints]
        trajectory_msg.points[0].time_from_start = rospy.Duration(3)
        
        rospy.sleep(1)
        trajectory_publisher.publish(trajectory_msg)
        rospy.sleep(3)  
        move_gripper(gripper_positions[idx], gripper_publisher)

    rospy.loginfo("Trajectory movement finished")

def move_gripper(position, publisher):
    rospy.loginfo("Moving gripper to position: {}".format(position))
    gripper_msg = GripperCommandActionGoal()
    gripper_msg.goal.command.position = position
    gripper_msg.goal.command.max_effort = 0.0
    publisher.publish(gripper_msg)
    rospy.sleep(1) 

if __name__ == '__main__':
    perform_trajectory()
