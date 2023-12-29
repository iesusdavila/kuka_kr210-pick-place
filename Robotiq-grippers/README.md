[![ROS](http://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png)](http://www.ros.org/)

<h1 style="border:none"> RISE ABB Robotiq Grippers ROS Manipulation Package </h1>
&copy; 2020, Francisco Yumbla

<hr>

## 1. How to Install

### 1.1. System Requirements

This package is written an tested on **Ubuntu 18.04 + ROS Melodic** environment. Dependencies are also for this environment.

Note: the same package can work in **Ubuntu 16.04 + ROS Kinetic**

### 1.2. Dependencies Prerequisites

There are a number of dependencies in this package, since the ABB robot is operated by ROS-Industrial package. Please install all the packages listed below in your Ubuntu PC, in the given order. These packages can be installed by `apt` package manager.

* ros-melodic-desktop-full
* ros-melodic-joint-state-publisher-gui

Now,Extract the metapackage `Robotiq-grippers` into `${ros_workspace}/src`. `catkin_make` your workspace.

## 2. Structure of Packages

* **robotiq_2f_85_description:** This package contains the URDF and XACRO files of the two finger paralell gripper.
* **robotiq_2f_gripper_control:** This package contains the control of the two finger gripper using a program or get the movement from `\joint_states`.
* **robotiq_3f_description:** This package contains the URDF and XACRO files of the three finger gripper.
* **robotiq_3f_gripper_control:** This package contains the control of the thre finger gripper using a program or get the movement from `\joint_states`.
* **grippers_communication:** This folders contains the package communications with the real grippers.

## 3. How to Use

### 3.1. Simulation

Open terminal and `roscore` and `Enter`. 


Launch the robot visualization Rviz

   ```
   roslaunch robotiq_2f_85_description display.launch
   ```

   ```
   roslaunch robotiq_3f_description display.launch
   ```


### 3.2. Real Robot

Connect the Gripper with DC power and the USB and turn on. 

Open terminal and `sudo chmod 666 /dev/ttyUSB0` (You need know wich is the port of the gripper `ls /dev/tty*`)



Launch the communication and the control of the gripper
   ```
   roslaunch robotiq_2f_gripper_control robotiq_2f_gripper_control.launch usb:=/dev/ttyUSB0
   ```

Launch the communication and the control of the gripper
   ```
   roslaunch robotiq_2f_gripper_control robotiq_3f_gripper_control.launch usb:=/dev/ttyUSB0
   ```

You can used the Simulation part to publish the respective `\joint_states`


### 3.2. Real Robot with default programs

if you want control the basic and example control
   ```
   robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py 
   robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py 
   ```
   ```
   rosrun robotiq_3f_gripper_control Robotiq3FGripperRtuNode.py
   rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py
   ```