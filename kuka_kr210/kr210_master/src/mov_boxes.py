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
        self.path = self.rospack.get_path('kr210_gazebo')+"/urdf/boxes/"
        self.boxes = []
        self.name_box = None
        self.box_count = 0
        self.boxes.append(self.path+"box_red.urdf")
        self.boxes.append(self.path+"box_green.urdf")
        self.boxes.append(self.path+"box_blue.urdf")

        self.red_count = 0
        self.green_count = 0
        self.blue_count = 0
        self.max_boxes = 5

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
        if self.red_count >= self.max_boxes and self.green_count >= self.max_boxes and self.blue_count >= self.max_boxes:
            rospy.loginfo("Maximum number of boxes reached for all colors.")
            return

        box = random.choice(self.boxes)
        box_color = box.split('/')[-1].split('_')[1]

        rospy.loginfo("Spawning a {} box.".format(box_color.split('.')[0]))
        if box_color.split('.')[0] == 'red' and self.red_count >= self.max_boxes:
            return
        elif box_color.split('.')[0] == 'green' and self.green_count >= self.max_boxes:
            return
        elif box_color.split('.')[0] == 'blue' and self.blue_count >= self.max_boxes:
            return

        with open(box, "r") as f:
            cube_urdf = f.read()

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(Point(x=0.8, y=-2.5, z=0.73), orient)

        # Generate a box with unique name
        self.box_count += 1
        new_name_box = f"box_{self.box_count}"

        self.sm(new_name_box, cube_urdf, '', pose, 'world')
        rospy.sleep(1)

        if box_color.split('.')[0] == 'red':
            self.red_count += 1
        elif box_color.split('.')[0] == 'green':
            self.green_count += 1
        elif box_color.split('.')[0] == 'blue':
            self.blue_count += 1

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

            if bottle.getPositionZ() < 0.6127:
                print("New Box Generate!!")
                bottle.spawnModel()
                conveyor_control.moveConveyor(60.0)  

            r.sleep()
    except KeyboardInterrupt or rospy.ROSInterruptException:
        conveyor_control.moveConveyor(0.0)  
        print("STOP!")
