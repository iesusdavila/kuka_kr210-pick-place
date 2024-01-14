#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from kr210_master.msg import DetectColor

class RobotController:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        self.arm = moveit_commander.MoveGroupCommander("kuka_arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        rospy.Subscriber("box_reached", DetectColor, self.box_reached_callback)
        self.is_box_reached = False
        self.color_detected = None

    def box_reached_callback(self, msg):
        self.is_box_reached = msg.detect
        self.color_detected = msg.color

    def move_to_position(self, translation, orientation=[3.14159,0,0]):
        pose_target = geometry_msgs.msg.Pose()
        q = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w = q
        pose_target.position.x, pose_target.position.y, pose_target.position.z = translation

        self.arm.set_pose_target(pose_target)
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    def move_gripper(self, position):
        current_joints = self.gripper.get_current_joint_values()
        current_joints[0] = position
        self.gripper.set_joint_value_target(current_joints)
        self.gripper.go(wait=True)
        self.gripper.stop()
        self.gripper.clear_pose_targets()

def perform_trajectory(robot_controller):
    home_position = [2, 0, 2]
    pick_position = [-0.021, -2.5, 1.085]
    place_position_red = [2.5, -0.4, 1.085]
    place_position_green = [-2.5, 0.4, 1.085]
    place_position_blue = [0.4, 2.5, 1.085]

    cont_box_red = 0
    cont_box_green = 0
    cont_box_blue = 0

    gripper_close = 0.2
    gripper_open = 0.0

    time_to_wait = 2.0

    rospy.loginfo("Initiating the trajectory movement")

    while not rospy.is_shutdown():
        if robot_controller.is_box_reached:
            robot_controller.is_box_reached = False
            compl_mov_color = False

            robot_controller.move_to_position(pick_position)
            robot_controller.move_gripper(gripper_close)
            rospy.sleep(time_to_wait)

            if robot_controller.color_detected == "Red":
                cont_box_red += 1
                if cont_box_red != 1:
                    place_position_red[1] = place_position_red[1] + 0.2
                    robot_controller.move_to_position(place_position_red)
                else:
                    robot_controller.move_to_position(place_position_red)
            elif robot_controller.color_detected == "Green":
                cont_box_green += 1
                if cont_box_green != 1:
                    place_position_green[1] = place_position_green[1] - 0.2
                    robot_controller.move_to_position(place_position_green)
                else:
                    robot_controller.move_to_position(place_position_green)
            elif robot_controller.color_detected == "Blue":
                cont_box_blue += 1
                if cont_box_blue != 1:
                    place_position_blue[0] = place_position_blue[0] - 0.2
                    robot_controller.move_to_position(place_position_blue, [3.14159, 0, 1.5708])
                else:
                    robot_controller.move_to_position(place_position_blue, [3.14159, 0, 1.5708])
            robot_controller.move_gripper(gripper_open)
            rospy.sleep(time_to_wait)

            robot_controller.move_to_position(home_position)

        else:
            robot_controller.move_to_position(home_position)
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        perform_trajectory(robot_controller)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")
    finally:
        moveit_commander.roscpp_shutdown()
