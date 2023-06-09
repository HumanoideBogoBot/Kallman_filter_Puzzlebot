#!/usr/bin/env python

#Santiago Ortiz Suzarte 		   A01750402
#Daniel Fuentes Castro			A01750425
#Leonardo Gracida Munoz 		A01379812
#Importamos las librerias necesarias
import tf, rospy
import numpy as np
from std_msgs.msg import Float32,Bool
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist, PointStamped,Quaternion,Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Kallman():
    def __init__(self):
        #iniciamos el nodo
        rospy.init_node('puzzlebot_kinematic_model')
        self.tb = tf.TransformBroadcaster()
        #Declaramos el objeto del broadcaster
        self.tb = tf.TransformBroadcaster()
        #Declaramos los publicadores necesarios
        self.pPoseCov = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.pPoseGaze = rospy.Publisher('/odom_gaze', Odometry, queue_size=10)
        self.pWl   = rospy.Publisher('/rviz/wl', Float32, queue_size=10)
        self.pWr   = rospy.Publisher('/rviz/wr', Float32, queue_size=10)
        self.pJS = rospy.Publisher('/joint_states', JointState,queue_size=10)
        self.actual_point_pub = rospy.Publisher('/actual_point', PointStamped,queue_size=10)

        #Declaramos los suscribers necesarios
        rospy.Subscriber("/cmd_vel", Twist, self.callback_top)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_gazebo_pose)
        rospy.Subscriber("/wl", Float32, self.callback_wl)
        rospy.Subscriber("/wr", Float32, self.callback_wr)
        rospy.Subscriber("/punto", PointStamped, self.callback_punto)
        rospy.Subscriber("/estado_aruco", Bool, self.arcuo_estado_callback)
        rospy.Subscriber("/euclidian_point", PointStamped, self.eclu_estado_callback)
        #Numero de mensajes por minuto
        self.fs = 50
        self.rate = rospy.Rate(self.fs)
        self.x_real = 0.0
        self.y_real = 0.0
        self.q_real = 0.0
        self.r = 0.05
        self.l = 0.188
        self.v = 0.0
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.q = 0
        self.xa = 0.01
        self.ya = 0
        self.qa = 0
        self.wr = 0
        self.wl = 0
        self.punto = PointStamped()
        self.estado_aruco = False
        self.z_p = 0
        self.z_alfa = 0
        self.z_k = np.array([[0],
                        [0]])
        self.z_hat_p = 0
        self.z_hat_alfa = 0
        self.z_k_hat = np.array([[0],
                        [0]])
        self.dis_eclu = 0
        self.alfa = 0
        self.x_gaze = 0
        self.y_gaze = 0
        self.q_gaze = [0,0,0,0]
    
    #Callback de la posicion del robot en gazebo
    def callback_gazebo_pose(self,data):
        self.x_gaze = data.pose[-1].position.x
        self.y_gaze = data.pose[-1].position.y
        self.q_gaze = [data.pose[-1].orientation.x,data.pose[-1].orientation.y,data.pose[-1].orientation.z,data.pose[-1].orientation.w]
    #Callback de la posicion del aruco detectado
    def callback_punto(self,msg):
        self.xa = msg.point.x
        self.ya = msg.point.y
        self.qa = msg.point.z
    #Callback de la distancia euclidiana  angulo de inclinacion de los arucos
    def eclu_estado_callback(self,msg):
        self.dis_eclu = msg.point.x
        self.alfa = msg.point.y
    #Callback que extrae la informacion de velocidad del topioc /cmd_vel
    def callback_top(self,msg):
        self.v=msg.linear.x
        self.w=msg.angular.z
    #Callback de la veloidad de loas llantas del robot
    def callback_wl(self,msg):
        self.wl = msg.data
    
    def callback_wr(self,msg):
        self.wr = msg.data

    def arcuo_estado_callback(self,msg):
        self.estado_aruco = msg.data


#Funcion que corre tanto la mejor posicion como publica la poisicion del robot de gazebo en RVIZ
    def node(self):
        tin = rospy.get_rostime().to_sec()
        #Declaramos el tamano de la diferencial de tiempo
        T = 1.0/self.fs
        t0 = rospy.Time.now()
        #Extraemos los angulos de inclinacion iniciales o anteriores para el calculo de la covarianza
        th_k_1 = self.q
        s_th_k_1 = self.q_real
        #Creamos la matriz de zeros de 3 y 6 dimensioness
        sigma_k_1 = np.zeros((3,3))
        sigma_com = np.zeros((6,6))
        aruco_x = 0.0001
        aruco_y = 0.0001
        while not rospy.is_shutdown():
            wr_ideal = self.v + 0.5*self.l*self.w
            wl_ideal = self.v - 0.5*self.l*self.w
            #Hacemos el calculo de la mejor aproximacion
            self.x += (T)*(self.v)*np.cos(self.q)
            self.y += (T)*(self.v)*np.sin(self.q)
            self.q += (T)*(self.w)
            #Real calculo de posicion con ruido
            v_real = self.r*((self.wr + self.wl)/2)
            w_real = self.r*((self.wr - self.wl)/self.l)
            self.x_real += (T)*(v_real)*np.cos(self.q_real)
            self.y_real += (T)*(v_real)*np.sin(self.q_real)
            self.q_real += (T)*(w_real)
            tfin = rospy.get_rostime().to_sec()
            #Transformamos el angulo de giro de euler a quaternion
            qRota = tf.transformations.quaternion_from_euler(0,0,self.q)
            cTime = rospy.Time.now()
            #Publicamos la posicion del gazebo en RVIZ
            self.tb.sendTransform([self.x_gaze,self.y_gaze,self.r], self.q_gaze, cTime, "base_link", "map")
            #Publicamos la velocidad de las llantas basandonos en la velocidad angular y lineal actual
            self.pWl.publish((self.v - 0.5*self.l*self.w)/self.r)
            self.pWr.publish((self.v + 0.5*self.l*self.w)/self.r)
            #Creamos el objeto para publicar estado de los joints de las llantas
            js = JointState()
            js.name = ["right_wheel_joint","left_wheel_joint"]
            t = cTime-t0
            js.position = [((self.v + 0.5*self.l*self.w)/self.r)*t.to_sec(),((self.v - 0.5*self.l*self.w)/self.r)*t.to_sec()]
            js.header.stamp = cTime
            #Publicamos el giro de las llantas
            self.pJS.publish(js)
            #Constantes de error de las llantas
            kr = 95
            kl = 95
            #Calculamos la matriz de covarianza de la posicion
            sigma_nabla_k = np.array([[kr*np.abs(self.wr),0],
                                      [0,kl*np.abs(self.wl)]])
            nabla_wk = 0.5*self.r*T*np.array([[np.cos(s_th_k_1),np.cos(s_th_k_1)],
                                               [np.sin(s_th_k_1),np.sin(s_th_k_1)],
                                               [2.0/self.l,-2.0/self.l]])
            Qk = np.dot(nabla_wk,np.dot(sigma_nabla_k,nabla_wk.T))
            Hk = np.array([[1,0,-T*self.v*np.sin(th_k_1)],
                           [0,1,T*self.v*np.cos(th_k_1)],
                           [0,0,1]])
            sigma_k = np.dot(Hk,np.dot(sigma_k_1,Hk.T))+Qk
            #En caso de obtener el estado de ver un aruco hacemos los calculos de Kalman
            if self.estado_aruco == True:
                #Kallman
                #Mediciones obtenidos de la camara y lobreria de fiducials
                self.z_p = self.dis_eclu
                self.z_alfa = self.alfa
                self.z_k = np.array([[self.z_p],
                                [self.z_alfa]])
                #Mediciones perfectas que deberia tener el robot
                self.z_hat_p = np.sqrt((self.xa - self.x)**2 + (self.ya - self.y)**2)
                self.z_hat_alfa = np.arctan2(self.ya - self.y,self.xa - self.x) - self.q
                self.z_k_hat = np.array([[self.z_hat_p],
                                [self.z_hat_alfa]])
                delta_x = self.xa - self.x
                delta_y = self.ya - self.y
                p = delta_x**2 + delta_y**2
                #Obtenemos el jacobiano del modelo de observacion
                Gk = np.array([[-delta_x/np.sqrt(p),-delta_y/np.sqrt(p),0],
                            [delta_y/p,-delta_x/p,-1]])
                #Error de podicion en inclinacion del aruco
                Rk = np.array([[0.1,0],
                            [0,0.05]])
                #Covarianza del modelo de observacion
                Z_k = np.dot(Gk,np.dot(sigma_k,Gk.T)) + Rk
                #Constante de Kalman
                Kk = np.dot(sigma_k,np.dot(Gk.T,np.linalg.inv(Z_k)))
                #Corregimos posicion
                fi_k = np.array([[self.x],[self.y],[self.q]]) + np.dot(Kk,(self.z_k - self.z_k_hat))
                #Corregimos covarianza
                sigma_k = np.dot((np.eye(3) - np.dot(Kk,Gk)),sigma_k)
            else:
                #En caso de no ver aruco no corregimos nada
                fi_k = np.array([[self.x],[self.y],[self.q]])

            #Cambiamos los valores de la matriz de covarianza de tres dimensiones
            sigma_com[0,0] = sigma_k[0,0]
            sigma_com[1,0] = sigma_k[1,0]
            sigma_com[5,0] = sigma_k[2,0]
            sigma_com[0,1] = sigma_k[0,1]
            sigma_com[1,1] = sigma_k[1,1]
            sigma_com[5,1] = sigma_k[2,1]
            sigma_com[0,5] = sigma_k[0,2]
            sigma_com[1,5] = sigma_k[1,2]
            sigma_com[5,5] = sigma_k[2,2]
            sigma_com[2,2] = 0.0001
            #Publicamos el mensaje de odometria
            x_fi = fi_k[0,0]
            y_fi = fi_k[1,0]
            q_fi = fi_k[2,0]
            q_fi = np.arctan2(np.sin(q_fi),np.cos(q_fi))
            dist_kall = np.sqrt((self.xa - x_fi)**2 + (self.ya - y_fi)**2)
            self.punto.header.frame_id = "map"
            self.punto.header.stamp = cTime
            self.punto.point = Point(x_fi,y_fi,q_fi)
            self.actual_point_pub.publish(self.punto)
            #Publicamos la posicion de Kallman en RVIZ
            locationCovariance = Odometry()
            locationCovariance.header.frame_id = "map"
            locationCovariance.header.stamp = cTime
            locationCovariance.child_frame_id = "base_kallman"
            locationCovariance.pose.pose.position = Point(x_fi,y_fi,self.r)
            q_fi_cuater = quaternion_from_euler(0,0,q_fi)
            locationCovariance.pose.pose.orientation = Quaternion(q_fi_cuater[0],q_fi_cuater[1],q_fi_cuater[2],q_fi_cuater[3])
            locationCovariance.pose.covariance = sigma_com.reshape(1,36)[0]
            locationCovariance.twist.twist.linear.x = self.v
            locationCovariance.twist.twist.linear.y = 0
            locationCovariance.twist.twist.linear.z = 0
            locationCovariance.twist.twist.angular.x = 0
            locationCovariance.twist.twist.angular.y = 0
            locationCovariance.twist.twist.angular.z = self.w
            locationCovariance.twist.covariance = np.zeros((6,6)).reshape(1,36)[0]
            self.pPoseCov.publish(locationCovariance)
            #Hacemos que los valores actuales se vuelvan los anteriores para la siguiente interacion
            self.x = x_fi
            self.y = y_fi
            self.q = q_fi
            sigma_k_1 = sigma_k
            th_k_1 = q_fi
            s_th_k_1 = self.q_real
            self.rate.sleep()
            tin = tfin

if __name__ == '__main__':
    try:
        kal = Kallman()
        kal.node()
    except rospy.ROSInterruptException:
    	pass

