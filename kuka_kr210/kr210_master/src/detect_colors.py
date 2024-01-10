#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        processed_image, color_detected = self.process_image(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        if color_detected:
            print(f"Detect color: {color_detected}")

    def process_image(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])

        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        red_output = cv2.bitwise_and(cv_image, cv_image, mask=mask_red)
        green_output = cv2.bitwise_and(cv_image, cv_image, mask=mask_green)
        blue_output = cv2.bitwise_and(cv_image, cv_image, mask=mask_blue)

        if np.any(red_output):
            return red_output, "Red"
        elif np.any(green_output):
            return green_output, "Green"
        elif np.any(blue_output):
            return blue_output, "Blue"
        else:
            return cv_image, None  

if __name__ == '__main__':
    rospy.init_node('color_detector', anonymous=True)
    cd = ColorDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
