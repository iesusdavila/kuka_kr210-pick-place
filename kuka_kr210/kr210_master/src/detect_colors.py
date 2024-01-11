#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from kr210_master.msg import DetectColor 
from cv_bridge import CvBridge, CvBridgeError

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera1/processed_image", Image, queue_size=10)
        self.box_reached_pub = rospy.Publisher("/box_reached", DetectColor, queue_size=10)
        self.black_point = (320, 248)

    def image_callback(self, data):
        # Convert the image from OpenCV to ROS format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Process the image to detect the color
        processed_image, color_detected = self.process_image(cv_image)

        if color_detected:
            print(f"Color detectado: {color_detected}")

            # Validate if the box has reached the black point and publish in its topic
            box_reached, cx, cy = self.is_box_at_point(processed_image)

            # Generate the custom message DetectColor to publish
            box_pub = DetectColor()
            box_pub.color = color_detected
            box_pub.detect = box_reached
            self.box_reached_pub.publish(box_pub)

            if box_reached:
                print("La caja ha llegado al punto negro.")
            
            cv2.circle(processed_image, (cx,cy), 5, [255, 255, 255], -1)
            cv2.circle(processed_image, self.black_point, 5, [255, 255, 255], -1)
        else:
            processed_image = np.zeros((cv_image.shape[0], cv_image.shape[1], 3), dtype=np.uint8)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def is_box_at_point(self, mask):
        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        
        _, thresh = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
        
        M = cv2.moments(thresh)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            if abs(cx - self.black_point[0]) < 10 and abs(cy - self.black_point[1]) < 10:
                return True, cx, cy
            else:
                return False, cx, cy
        return False, None, None

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
