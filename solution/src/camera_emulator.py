#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv_bridge

class CameraEmulator():

    def __init__(self):
        rospy.init_node("camera_emulator")
        self.img_pub_output = rospy.Publisher("/video_source/raw",Image,queue_size=1)
        self.bridge = cv_bridge.CvBridge()
        self.img_sub = rospy.Subscriber("/camera/image_raw",Image,self.imgCallback)
        self.frame = np.array([[]],dtype="uint8")
        self.rate = rospy.Rate(60)

    def imgCallback(self,data):
        # callback for the img from camera
        try:
            frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            self.frame = frame
        except cv_bridge.CvBridgeError():
            print("Error CvBridge")
    
    def transformation(self):
        # copy of image subscriber
        frame = self.frame
        frame = cv2.resize(frame, (320,280), interpolation=cv2.INTER_NEAREST)
        self.img_pub_output.publish(self.bridge.cv2_to_imgmsg(frame,"bgr8"))
    
    def main(self):
        # main execution while node runs
        print("Running main...")
        while not rospy.is_shutdown():
            try:
                self.transformation()
            except Exception as e:
                print("wait")
                print(e)
            self.rate.sleep()

if __name__ == "__main__":
	try:
		node = CameraEmulator()
		node.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print('topic was closed during publish')