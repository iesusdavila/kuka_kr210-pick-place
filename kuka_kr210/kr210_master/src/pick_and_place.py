#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool 

class RobotArmClient:
    def __init__(self, arm_controller_name, gripper_controller_name):
        self.arm_client = actionlib.SimpleActionClient(arm_controller_name, FollowJointTrajectoryAction)
        self.gripper_client = actionlib.SimpleActionClient(gripper_controller_name, GripperCommandAction)
        self.box_reached_sub = rospy.Subscriber("/box_reached", Bool, self.box_reached_callback)
        self.is_box_reached = False

        rospy.loginfo("Waiting for the arm controller server...")
        self.arm_client.wait_for_server()
        rospy.loginfo("Waiting for the gripper controller server...")
        self.gripper_client.wait_for_server()

        self.kuka_joints = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
    
    def box_reached_callback(self, data):
        self.is_box_reached = data.data

    def move_to_joint_positions(self, positions):
        rospy.loginfo("Moving the arm to the joint positions")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.kuka_joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration.from_sec(5.0))

        if self.arm_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Moving the arm to the joint positions completed successfully")
            return True
        else:
            rospy.loginfo("Moving the arm to the joint positions failed")
            return False

    def move_gripper(self, position):
        rospy.loginfo("Moving the gripper to the position: {}".format(position))
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = 0.0

        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

        if self.gripper_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Moving the gripper to the position completed successfully")
            return True
        else:
            rospy.loginfo("Moving the gripper to the position failed")
            return False

def perform_trajectory(arm_client):
    pick_positions = [-1.57, 0.62, -0.349066, 0, 1.320, -1.5708]
    place_positions = [0, 0, 0, 0, 0, 0.0]

    gripper_close = 0.18
    gripper_open = 0.0

    rospy.loginfo("Initiating the trajectory movement")

    while not rospy.is_shutdown():
        if arm_client.is_box_reached:
            arm_client.is_box_reached = False

            rospy.loginfo("Moving the arm to the pick up position")
            if arm_client.move_to_joint_positions(pick_positions):
                rospy.loginfo("Closing the gripper")
                arm_client.move_gripper(gripper_close)
                rospy.sleep(1.0)
            
            rospy.loginfo("Moving the arm to the place position")
            arm_client.move_to_joint_positions(place_positions)
            rospy.loginfo("Opening the gripper")
            arm_client.move_gripper(gripper_open)
            rospy.sleep(1.0)
        
        rospy.sleep(0.1) 

    rospy.loginfo("Moving of the trajectory finished")

if __name__ == '__main__':
    try:
        rospy.init_node('kuka_trajectory_publisher')
        arm_controller_name = '/arm_controller/follow_joint_trajectory'
        gripper_controller_name = '/gripper/gripper_cmd'
        arm_client = RobotArmClient(arm_controller_name, gripper_controller_name)
        perform_trajectory(arm_client)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")