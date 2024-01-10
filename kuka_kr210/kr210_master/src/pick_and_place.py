#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def perform_trajectory():

    rospy.init_node('kuka_trajectory_publisher')  # Node definition

    controller_name = '/arm_controller/command'
    trajectory_publisher = rospy.Publisher(controller_name, JointTrajectory, queue_size=10)  # Publisher Definition

    kuka_joints = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']  # Name of joints

    # Lista de puntos de trayectoria predefinidos
    waypoints = [
        [-1.57, 0.61, -0.349066, 0, 1.309, -1.5708],
        [0, 0, 0, 0, 0, 0.75],
        [-1.57, 0.61, -0.349066, 0, 1.309, -1.5708],
        [0, 0, 0, 0, 0, 0.75]
    ]

    rospy.loginfo("Starting trajectory movement")

    for point in waypoints:
        trajectory_msg = JointTrajectory()  # JointTrajectory object declaration
        trajectory_msg.joint_names = kuka_joints

        # Se añade el punto de trayectoria actual
        trajectory_msg.points.append(JointTrajectoryPoint())
        trajectory_msg.points[0].positions = point
        trajectory_msg.points[0].velocities = [0.0 for i in kuka_joints]
        trajectory_msg.points[0].accelerations = [0.0 for i in kuka_joints]
        trajectory_msg.points[0].time_from_start = rospy.Duration(3)

        rospy.sleep(1)
        trajectory_publisher.publish(trajectory_msg)
        rospy.sleep(3)  # Espera a que se complete la trayectoria actual antes de continuar

    rospy.loginfo("Trajectory movement finished")


if __name__ == '__main__':
    try:
        perform_trajectory()
    except rospy.ROSInterruptException:
        pass