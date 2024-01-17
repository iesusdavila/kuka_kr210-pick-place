#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from collections import defaultdict

positions = defaultdict(list)
velocities = defaultdict(list)
times = []

def joint_state_callback(msg):
    time = rospy.get_time()
    times.append(time)

    for i, name in enumerate(msg.name):
        if name in ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']:
            positions[name].append(msg.position[i])
            velocities[name].append(msg.velocity[i])

def plot_joint_data():
    plt.figure(figsize=(10, 6))
    for joint in positions:
        plt.plot(times, positions[joint], label=joint)
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.title('Joints Positions')
    plt.legend()
    plt.show()

    plt.figure(figsize=(10, 6))
    for joint in velocities:
        plt.plot(times, velocities[joint], label=joint)
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title('Joints Speeds')
    plt.legend()
    plt.show()

def main():
    rospy.init_node('joint_state_listener', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    rospy.spin()

    plot_joint_data()

if __name__ == '__main__':
    main()
