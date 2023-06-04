#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped,Point
import cv_bridge

class ArucoDetector():

    def __init__(self):
        # constructor node publishers and subscribers
        rospy.init_node("sign_detector")
        self.img_pub_output = rospy.Publisher("/processed_image/output",Image,queue_size=1)
        self.point_publisher = rospy.Publisher("/punto",PointStamped,queue_size=1)
        self.bridge = cv_bridge.CvBridge()
        self.img_sub = rospy.Subscriber("/camera/image_raw",Image,self.imgCallback)
        self.rate = rospy.Rate(60)
        self.frame = np.array([[]],dtype="uint8")
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50) # must match generated aruco dict
        self.aruco_params = cv2.aruco.DetectorParameters_create()# aruco parameters object
        #self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.puntos = [(1,0.0),(1,0.5),(6,0),(8,-1),(10,0),(8,3),(7,2),(4.5,3),(2,2),(6,4)]
        self.punto_aruco = PointStamped()
        self.estado_inicial = True
        self.distance_eclu = 0

    def imgCallback(self,data):
        # callback for the img from camera
        try:
            frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            self.frame = frame
        except cv_bridge.CvBridgeError():
            print("Error CvBridge")

    def processImg(self):

        # copy of image subscriber
        frame = self.frame
        mtx = np.array([[206.66031,    0.     ,  162.36607]   ,
                        [0.     ,  322.1899 ,  145.66166]    ,
                        [0.     ,    0.     ,    1.     ]])
        dist = np.array([[-0.347695, 0.123583, -0.001230, 0.000523, 0.000000]])
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (320,280), 1, (320,280))
        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        #frame = dst[y:y+h, x:x+w]
        (corners,ids,rejected) = cv2.aruco.detectMarkers(frame,self.aruco_dict,parameters=self.aruco_params)
        markerSizeInCM = 16.4
        rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, mtx, dist)
        if tvec is not None:
            #print(tvec,np.degrees(rvec[0][0][2]))
            self.distance_eclu = tvec[0][0][2]
        #cv2.aruco.detectMarkers(frame,self.aruco_dict,parameters=self.aruco_params)
        #rvec, tvec, markerPoints = cv2.solvePnP(corners, 0.02)
        # print('corners',corners)
        # print('ids',ids)
        # print('rejected',rejected)

        # check at least 1 detection
        if (len(corners) <= 0) and (self.estado_inicial == True):
            cTime = rospy.Time.now()
            self.punto_aruco.header.frame_id = "map"
            self.punto_aruco.header.stamp = cTime
            self.punto_aruco.point = Point(0,0,0)

        if len(corners) > 0:
            self.estado_inicial = False
            # flatten markers ids [list]
            ids = ids.flatten()
            # extract and draw
            for (markerCorner,markerID) in zip(corners,ids):
                # corners returned in the way:
                # top-left,top-right,bottom-right,bottom-left
                corners = markerCorner.reshape((4,2))
                (topLeft,topRight,bottomRight,bottomLeft) = corners
                x = np.abs(topRight[1] - bottomRight[1])
                y = np.abs(topRight[0] - topLeft[0])
                area = x*y
                #print(area)
                # order corners (x,y) pairs per line
                topRight = (int(topRight[0]),int(topRight[1]))
                bottomRight = (int(bottomRight[0]),int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]),int(bottomLeft[1]))
                topLeft = (int(topLeft[0]),int(topLeft[1]))
                # draw enclosing rectangle
                cv2.line(frame,topLeft,topRight,(0,255,0),1)
                cv2.line(frame,topRight,bottomRight, (0,255,0),1)
                cv2.line(frame,bottomRight,bottomLeft,(0,255,0),1)
                cv2.line(frame,bottomLeft,topLeft,(0,255,0),1)
                # draw centroid
                cX = int((topLeft[0]+bottomRight[0])/2.0)
                cY = int((topLeft[1]+bottomLeft[1])/2.0)
                cv2.circle(frame,(cX,cY),4,(0,0,255),-1)
                # draw aruco ID as text
                cv2.putText(frame,str(markerID),(topLeft[0],topLeft[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                cTime = rospy.Time.now()
                self.punto_aruco.header.frame_id = "map"
                self.punto_aruco.header.stamp = cTime
                self.punto_aruco.point = Point(self.puntos[int(markerID)][0],self.puntos[int(markerID)][1],self.distance_eclu)
                self.point_publisher.publish(self.punto_aruco)

        self.img_pub_output.publish(self.bridge.cv2_to_imgmsg(frame,"bgr8"))
	
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
