#!/usr/bin/env python3

import rospy, tf, rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point
from gazebo_conveyor.srv import ConveyorBeltControl  # Import the correct service definition

class ConveyorControl():
    def __init__(self):
        self.conveyor_service = rospy.ServiceProxy('/conveyor/control', ConveyorBeltControl)

    def moveConveyor(self, power):
        try:
            response = self.conveyor_service(power)
            if response.success:
                rospy.loginfo("Conveyor moved successfully with power: %.1f", power)
            else:
                rospy.logwarn("Conveyor movement request failed.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))

class BoxSpawner():
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('kr210_gazebo')+"/urdf/"
        self.cubes = []
        self.cubes.append(self.path+"red_cube.urdf")
        self.cubes.append(self.path+"green_cube.urdf")
        self.cubes.append(self.path+"blue_cube.urdf")
        self.col = 0

        self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    def checkModel(self):
        res = self.ms("cube", "world")
        return res.success

    def getPositionX(self):
        res = self.ms("cube", "world")
        return res.pose.position.x

    def getPositionZ(self):
        res = self.ms("cube", "world")
        return res.pose.position.z

    def spawnModel(self):
        cube = self.cubes[self.col]
        with open(cube,"r") as f:
            cube_urdf = f.read()
        
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(Point(x=0.8, y=-2.5, z=0.73), orient) 
        self.sm("cube", cube_urdf, '', pose, 'world')
        
        if self.col < 2:
            self.col += 1
        else:
            self.col = 0
        rospy.sleep(1)

    def deleteModel(self):
        self.dm("cube")
        rospy.sleep(1)

    def shutdown_hook(self):
        self.deleteModel()
        print("Shutting down")

if __name__ == "__main__":
    try:
        print("Waiting for gazebo services...")
        rospy.init_node("spawn_cubes")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/get_model_state")
        r = rospy.Rate(15)
        bottle = BoxSpawner()
        conveyor_control = ConveyorControl()  # Create an instance of ConveyorControl
        rospy.on_shutdown(bottle.shutdown_hook)

        while not rospy.is_shutdown():

            print(f'Position X: {bottle.getPositionX()}')
            print(f'Position Z: {bottle.getPositionZ()}')
    
            if bottle.checkModel() == False:
                print("New Box!")
                bottle.spawnModel()
                conveyor_control.moveConveyor(60.0)  
            elif bottle.getPositionX() <= 0 or bottle.getPositionZ() < 0.1: 
                conveyor_control.moveConveyor(0.0)  

            if bottle.getPositionZ() < 0.1:
                bottle.deleteModel()

            r.sleep()
    except KeyboardInterrupt or rospy.ROSInterruptException:
        conveyor_control.moveConveyor(0.0)  
        print("STOP!")
