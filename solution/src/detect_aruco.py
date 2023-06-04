#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped,Point
from fiducial_msgs.msg import FiducialTransformArray
from nav_2d_msgs.msg import Point2D
import cv_bridge
class ArucoDetector():

    def __init__(self):
        # constructor node publishers and subscribers
        rospy.init_node("sign_detector")
        #self.img_pub_output = rospy.Publisher("/processed_image/output",Image,queue_size=1)
        self.point_publisher = rospy.Publisher("/punto",PointStamped,queue_size=1)
        self.estado_publisher = rospy.Publisher("/estado_aruco",Bool,queue_size=1)
        self.z_point_publisher = rospy.Publisher("/euclidian_point",PointStamped,queue_size=1)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback_fiducal)
        self.rate = rospy.Rate(60)
        self.frame = np.array([[]],dtype="uint8")
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50) # must match generated aruco dict
        self.aruco_params = cv2.aruco.DetectorParameters_create()# aruco parameters object
        #self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.puntos = [(1,0.0),(4,-1),(6,0),(8,-1),(9,0),(8,2),(7,1.5),(6,3),(4,4),(4,1.5),(0,1),(3,1.5),(2,3),(1,3)]
        self.punto_aruco = PointStamped()
        self.z_puntos = PointStamped()
        self.estado_inicial = True
        self.distance_eclu = 0
        self.arucos = []
    
    def callback_fiducal(self,msg):
        self.arucos = msg


    def processImg(self):
        Ry = np.array([[np.cos(np.degrees(-90)),0,np.sin(np.degrees(-90))],
                       [0,1,0],
                       [-np.sin(np.degrees(-90)),0,np.cos(np.degrees(-90))]])
        Rx = np.array([[1,0,0],
                       [0,np.cos(np.degrees(90)),-np.sin(np.degrees(90))],
                       [0,np.sin(np.degrees(90)),np.cos(np.degrees(90))]])
        #print(self.arucos.transforms[0])
        if len(self.arucos.transforms) > 0.0:
            for i in self.arucos.transforms:
                #print(i.fiducial_id)
                #print(i.transform.translation.x)
                #print(i.transform.translation.z)
                pos = np.array([[i.transform.translation.x],
                                [i.transform.translation.y],
                                [i.transform.translation.z]])
                #R = np.dot(Ry,Rx)
                #pos = np.dot(R,pos)
                #print(pos)
                x = i.transform.translation.z
                y = -i.transform.translation.x
                id = i.fiducial_id
                dis = np.sqrt(x**2 + y**2)
                alfa = np.arctan2(y,x)
                if(dis <= 2):
                    print(id-100,dis,np.degrees(alfa))
                    cTime = rospy.Time.now()
                    self.estado_publisher.publish(Bool(True))
                    self.punto_aruco.header.frame_id = "map"
                    self.punto_aruco.header.stamp = cTime
                    self.punto_aruco.point = Point(self.puntos[int(id-100)][0],self.puntos[int(id-100)][1],0)
                    self.point_publisher.publish(self.punto_aruco)

                    cTime = rospy.Time.now()
                    self.z_puntos.header.frame_id = "map"
                    self.z_puntos.header.stamp = cTime
                    self.z_puntos.point = Point(dis,alfa,0)
                    self.z_point_publisher.publish(self.z_puntos)
                else:
                    self.estado_publisher.publish(Bool(False))
        else:
            self.estado_publisher.publish(Bool(False))

        #frame = dst[y:y+h, x:x+w]
	
    def main(self):
        # main execution while node runs
        print("Running main...")
        while not rospy.is_shutdown():
            #print(self.punto_aruco)
            try:
                self.processImg()
            except Exception as e:
                print("wait")
                print(e)
            self.rate.sleep()
        # cv2.destroyAllWindows()

if __name__ == "__main__":
	try:
		node = ArucoDetector()
		node.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print('topic was closed during publish')
