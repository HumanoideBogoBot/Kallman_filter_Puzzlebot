#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

rospy.init_node('puzzlebot_kinematic_model')

msg_camera = CameraInfo()
imagePubliser = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)

while not rospy.is_shutdown():
    cTime = rospy.Time.now()
    msg_camera.header.stamp = cTime
    msg_camera.height = 280
    msg_camera.width = 320
    msg_camera.distortion_model = "plumb_bob"
    msg_camera.D = [-0.344805, 0.099477, 0.002612, 0.006285, 0.000000]
    msg_camera.K = [ 838.540632,    0.     ,  602.111656,
            0.     ,  840.749230 ,  358.952831,
            0.     ,    0.     ,    1.     ]
    msg_camera.R = [ 1.,  0.,  0.,
                    0.,  1.,  0.,
                    0.,  0.,  1.]
    msg_camera.P = [ 632.613770,    0.     ,  604.100740,    0.     ,
            0.     ,  783.156860,  360.405715 ,    0.     ,
            0.     ,    0.     ,    1.     ,    0.     ]
    msg_camera.binning_x = 0
    msg_camera.binning_y = 0
    msg_camera.roi.x_offset = 0
    msg_camera.roi.y_offset = 0
    msg_camera.roi.height = 0
    msg_camera.roi.width = 0
    msg_camera.roi.do_rectify = False
    imagePubliser.publish(msg_camera)