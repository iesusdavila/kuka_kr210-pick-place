#!/usr/bin/env python3

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point
from gazebo_conveyor.srv import ConveyorBeltControl  

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
        self.boxes = []
        self.name_box = None
        self.boxes.append(self.path+"box_red.urdf")
        self.boxes.append(self.path+"box_green.urdf")
        self.boxes.append(self.path+"box_blue.urdf")

        self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    def checkModel(self):
        if self.name_box is None:
            return False  
        res = self.ms(self.name_box, "world")
        return res.success

    def getPositionX(self):
        if self.name_box is None:
            return None
        res = self.ms(self.name_box, "world")
        return res.pose.position.x

    def getPositionZ(self):
        if self.name_box is None:
            return None
        res = self.ms(self.name_box, "world")
        return res.pose.position.z

    def spawnModel(self):
        box = random.choice(self.boxes)  
        with open(box,"r") as f:
            cube_urdf = f.read()
        
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(Point(x=0.8, y=-2.5, z=0.73), orient)

        new_name_box = box.split('/')[-1].split('.')[0]
        self.sm(new_name_box, cube_urdf, '', pose, 'world')
        rospy.sleep(1)  

        self.name_box = new_name_box  

    def deleteModel(self):
        if self.name_box is None:
            return
        self.dm(self.name_box)
        self.name_box = None  
        rospy.sleep(1)

    def shutdown_hook(self):
        self.deleteModel()
        print("Goodbye!")

if __name__ == "__main__":
    try:
        rospy.init_node("move_boxes")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/get_model_state")
        r = rospy.Rate(15)
        bottle = BoxSpawner()
        conveyor_control = ConveyorControl()
        rospy.on_shutdown(bottle.shutdown_hook)

        while not rospy.is_shutdown():    
            if bottle.checkModel() == False:
                print("New Box Generate!!")
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
