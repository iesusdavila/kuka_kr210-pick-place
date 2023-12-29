# Gazebo utils for gripper gazebo plugin

## Prerequisite:

1. Ubuntu 18.04 LTS
2. [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
3. [MoveIt!](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
4. [OpenNI Kinect Package (Camera)](https://www.oreilly.com/library/view/learning-robotics-using/9781788623315/1235f7fe-3637-412a-a386-05859b89ee67.xhtml)


## Getting Started:

### 1. Create new catkin workspace:

    mkdir -p catkin_ws/src
    
### 2. Git clone to the catkin_ws/src:
```
  cd catkin_ws/src

  git clone https://github.com/javierpagalo/Gazebo_utils.git   
```
### 3. Install dependencies using rosdep install and perform catkin_make to build the project:

   `cd catkin_ws` or `cd ..`
   
   `rosdep install --from-paths src --ignore-src -r -y`
   
   `catkin_make`
   
   source the file before launch
   
   `source devel/setup.bash`
   
    
## References
1. [grasp_plugin](https://github.com/JenniferBuehler/gazebo-pkgs.git)
2. [general_messages](https://github.com/JenniferBuehler/general-message-pkgs.git)
3. [pick_and_place_repo](https://github.com/karhong-sam/pick-and-place-with-icl-ur5-robotiq-gripper.git)

