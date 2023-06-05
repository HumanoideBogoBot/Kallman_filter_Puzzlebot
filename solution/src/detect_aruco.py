#!/usr/bin/env python
#Santiago Ortiz Suzarte 		   A01750402
#Daniel Fuentes Castro			A01750425
#Leonardo Gracida Munoz 		A01379812
import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped,Point
from fiducial_msgs.msg import FiducialTransformArray
class ArucoDetector():

    def __init__(self):
        # constructor node publishers and subscribers
        rospy.init_node("sign_detector")
        #Publishers and suscribers necesarios
        self.point_publisher = rospy.Publisher("/punto",PointStamped,queue_size=1)
        self.estado_publisher = rospy.Publisher("/estado_aruco",Bool,queue_size=1)
        self.z_point_publisher = rospy.Publisher("/euclidian_point",PointStamped,queue_size=1)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback_fiducal)
        self.rate = rospy.Rate(60)
        self.frame = np.array([[]],dtype="uint8")
        #Las posiciones de los arucos en orden del id en el que estan relacionados.
        self.puntos = [(1,0.0),(4,-1),(6,0),(8,-1),(9,0),(8,2),(7,1.5),(6,3),(4,4),(4,1.5),(0,1),(3,1.5),(2,3),(1,3)]
        self.punto_aruco = PointStamped()
        self.z_puntos = PointStamped()
        self.estado_inicial = True
        self.distance_eclu = 0
        self.arucos = []
    
    #Callback para hacer obtener la informacion de la libreria de los arucos
    def callback_fiducal(self,msg):
        self.arucos = msg


    def processImg(self):
        #Si detectamos un aruco:
        if len(self.arucos.transforms) > 0.0:
            for i in self.arucos.transforms:
                #Obtenemos la traslacion del aruco
                x = i.transform.translation.z
                y = -i.transform.translation.x
                id = i.fiducial_id
                #Obtenemos distancia euclidiana y angulo de inclinacion en el sistema de referecnai del robot
                dis = np.sqrt(x**2 + y**2)
                alfa = np.arctan2(y,x)
                if(dis <= 2):
                    cTime = rospy.Time.now()
                    #Publicamos True si detectamos un aruco cerca
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
                    #Publicamos False si detectamos un aruco lejos
                    self.estado_publisher.publish(Bool(False))
        else:
            #Publicamos False si no detectamos ningun aruco
            self.estado_publisher.publish(Bool(False))
	
    def main(self):
        #Funcion que corre la funcion principal
        print("Running main...")
        while not rospy.is_shutdown():
            try:
                self.processImg()
            except Exception as e:
                #Mostramos el error encontrado
                print("wait")
                print(e)
            self.rate.sleep()

if __name__ == "__main__":
	try:
		node = ArucoDetector()
		node.main()
	except (rospy.ROSInterruptException, rospy.ROSException):
		print('topic was closed during publish')
