#!/usr/bin/env python 
#Leonardo Gracida Munoz A01379812
#Daniel Fuentes Castro A01750425
#Santiago Ortiz Suzarte A01750402
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist,PointStamped,Point,Vector3,Quaternion
from std_msgs.msg import Float32,ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

class RightHandRuleController:
    def __init__(self, wall_dist=0.5, w_max = 0.15, v_max=0.1,puntos=[(0,0)]):
        #Declaramos los publishers y suscribers necesarios
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.wr_listener = rospy.Subscriber('/wr', Float32,self.wr_callback)
        self.wl_listener = rospy.Subscriber('/wl', Float32,self.wl_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        self.marker_pub = rospy.Publisher('/goals' , Marker, queue_size=1 )
        self.posicion_lis = rospy.Subscriber('/actual_point', PointStamped,self.point_callback)
        self.rate = rospy.Rate(50.0)
        self.puntos = puntos
        self.wall_dist = wall_dist
        self.lugar = 0
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None
        self.xt = 0
        self.yt = 0
        self.x = 0
        self.y = 0
        self.th = 0
        self.wr = 0
        self.wl = 0
        self.r =  0.05
        self.l = 0.188
        self.distance_ahead = 0 
        self.distance_to_right = 0
        self.rayo_izq = 0
        self.rayo_der = 0
        self.eth = 0
        self.xin = 0
        self.yin = 0
        self.thin = 0
        self.alpha = 0
        self.a = 0
        self.b = 0
        self.c = 0
        self.lugar = 0
        self.final = False
        self.marker = Marker()                  # line config
        self.marker.header.frame_id = 'map'
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale = Vector3(0.03,0.03,0.03)     # x,y,z
        self.marker.color = ColorRGBA(1.0,1.0,1.0,1.0)  # r,g,b,a
        self.marker.pose.position = Point(0.0,0.0,0.0)
        self.marker.pose.orientation = Quaternion(0.0,0.0,0.0,1.0)
        self.puntos_marker = []
        for i in puntos:
            self.puntos_marker.append(Point(i[0],i[1],0.0))
        self.marker.points = [Point(0.0,0.0,0.0)] + self.puntos_marker
        
    #Callback de lo medido con el LIDAR
    def scan_callback(self, msg):
        self.scan = msg
    
    #Posicion del robot calculado por Kalman
    def point_callback(self,msg):
        self.x = msg.point.x
        self.y = msg.point.y
        self.q = msg.point.z
    
    #Velocidades de los encoders del robot
    def wr_callback(self, msg):
        self.wr = msg.data
    def wl_callback(self, msg):
        self.wl = msg.data
    
    #Funcion seguidor de linea
    def follow_left_hand_wall(self):
        #print(len(self.scan.ranges))
        if self.scan is not None:
            #Obtenemos el angulo de inclinacion que tiene el robot conforme al muro                               
            alpha = find_wall_direction(self.scan)
            #Obtenemos la distancia perpendicular que tiene el robot conforme al muro
            perpendicular_wall_dis = self.distance_to_left * np.cos(alpha)
            #Declaramos el angulo de inclinacion Deseado
            alpha_des = 0
            #Declaramos a que distancia deseamos estar alejados del muro
            perpendicular_wall_dis_des = self.wall_dist
            #Obtenemos el error de angulo de distancia hacia el muro
            error_alpha = alpha_des - alpha
            error_perpendicular_wall_dis = perpendicular_wall_dis_des - perpendicular_wall_dis
            #Definimos las constantes proporcionales del sistema de control
            kp_alpha = 1
            kp_perpendicular_wall_dis = 1
            #Sumamos lo de los dos controladores proporcionales para obtener la velocidad angular
            w = (kp_alpha * error_alpha) + (kp_perpendicular_wall_dis * error_perpendicular_wall_dis)
            #Definimos el limte del filtro de saturacion
            limite = 1.5
            #Defimimos la distancia maxim que nos podemos hacercar de frente con el robot
            limite_dis_ahead = 1
            #Creamos el filtro de saturacion.
            v = self.v_max
            if w > self.w_max:
                w = self.w_max
            elif w < -1*self.w_max:
                w = -1*self.w_max
            #Si estamos muy cerca del muro giramosa la izquierda.
            if self.distance_ahead < limite_dis_ahead:
                w = self.w_max
            
            return (v,-w)
        else:
            return (0.0,0.0)
            
    
    def cal_odometry(self,th,dt):
        th = th + self.r*((self.wr-self.wl)/(self.l))*dt
        return (th)

    def odometry(self,dt,xin,yin):
        #Hacemos los nuevos calculos de la odometria
        """Aqui hacemos un angulo de inclinacion local para poder hacer un giro contratrio a las
        cajas para evitar que el robot la siga"""
        th = self.cal_odometry(self.th,dt)
        self.th = th
        print(self.xt,self.yt,xin,yin)
        #Obtenemos el giro deseado para alinearnos
        thi = np.arctan2(self.yt-yin,self.xt-xin)
        #Obtenemos el error de giro
        ther = thi - self.th
        self.eth = ther
        #Obtenemos la distancia euclidiana al punto final
        edis = np.sqrt((self.xt - self.x)**2 + (self.yt - self.y)**2)
        #Constantes de control para el giro y avanzar hacia el punto
        kd,kth = 1,1
        #Distancia hacia la linea central
        dist_lin = -((self.a*self.x + self.b*self.y + self.c))/(np.sqrt(self.a**2 + self.b**2))
        w = ther*kth + dist_lin * 8
        print("Error q:",ther)
        v = kd * edis
        #Filtro de saturacion
        if v > self.v_max:
                v = self.v_max
        elif v < -1*self.v_max:
            v = -1*self.v_max
        if w > self.w_max:
                w = self.w_max
        elif w < -1*self.w_max:
            w = -1*self.w_max
        #Si estamos cerca del punto paramos
        if (np.abs(w) > 0.05) and (dist_lin < 0.05):
            v = 0
        print(edis)
        #Si estamos en el ultimo punto cerca paramos
        if (edis < 0.09) and ((self.lugar+1) == len(self.puntos)):
            v = 0
            w = 0
            self.final = True
        #Si estamos cerca el punto cambiamos de punto
        elif (edis < 0.09):
            self.lugar += 1
            self.xt = self.puntos[self.lugar][0]
            self.yt = self.puntos[self.lugar][1]
            self.cal_linea(self.x,self.y,self.xt,self.yt)
            self.xin  = self.x
            self.yin = self.y

        return (v,w)

    def cal_linea(self,x0,y0,xf,yf):
        #Funcion que obtiene las constantes a,b,c para oobtener la desitancia con la linea original
        x0_inv = -xf
        y0_inv = -yf
        a,b,c = 0,0,0
        a = -(yf - y0)
        b = (xf -x0)
        m = (yf-y0)/(xf-x0)
        independiente = yf - m*xf
        c = -(xf-x0)*independiente
        self.a = a
        self.b = b
        self.c = c


        
    def main(self):
        #Obtenemos el tiempo inicial
        t0 = rospy.get_rostime().to_sec()
        #Estados que controlan si seguimos pared o seguimos odometria
        estado_odom = True
        estado_esquina = False
        #Obtenemos las constantes del la linea recta deseada
        self.xt = self.puntos[self.lugar][0]
        self.yt = self.puntos[self.lugar][1]
        self.cal_linea(self.x,self.y,self.xt,self.yt)
        x_antes_linea_ini = self.x
        y_antes_linea_ini = self.y
        estado_lin_inicio = False
        estado_lin_reset = False
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.marker)
            #Si el scan no esta vacio
            if self.scan is None:
                continue
            """Obtenemos la distancia hacia enfrente a la izquierda, derecha y el rayo de 45 grados de la izquierda y derecha"""
            self.distance_ahead = (get_distance_in_sector(self.scan,np.pi,np.pi - 2*np.pi/180)+get_distance_in_sector(self.scan,-np.pi + 2*np.pi/180,-np.pi))/2
            
            self.distance_to_left = get_distance_in_sector(self.scan,
                                                    -np.pi/2 + 4*np.pi/180,
                                                    -np.pi/2)
            self.distance_to_right = get_distance_in_sector(self.scan,
                                                    np.pi/2,
                                                    np.pi/2 - 4*np.pi/180)
            self.rayo_izq = get_distance_in_sector(self.scan,
                                                    np.radians(-133),
                                                    np.radians(-137))
            self.rayo_der = get_distance_in_sector(self.scan,
                                                    np.radians(137),
                                                    np.radians(133))
            tf = rospy.get_rostime().to_sec()
            #Obtenemos la diferencial de tiempos
            dt = tf- t0
            if dt > 0.1:
                dt = 0.0
            #Hacmos la odometria todo el tiempo para saber donde estamos
            print("iniciales:",self.xin,self.yin)
            v,w = self.odometry(dt,self.xin,self.yin)
            #Obtenemos la distancia con la linea original
            dist = (np.abs(self.a*self.x + self.b*self.y + self.c))/(np.sqrt(self.a**2 + self.b**2))
            dis_lin = np.sqrt((self.x - x_antes_linea_ini)**2 + (self.y - y_antes_linea_ini)**2)
            #Si detectamos alfrente o a los lamos mas cerca que un umbral pasamos a modo wall follower
            if (self.distance_ahead < 1):
                self.thin = self.th
                estado_odom = False
                if (dist <= 0.05) and (self.rayo_izq > 1):
                    x_antes_linea_ini = self.x
                    y_antes_linea_ini = self.y
            if (self.distance_ahead < 1):
                if (dist <= 0.05):
                    x_antes_linea_ini = self.x
                    y_antes_linea_ini = self.y
            #Si estamos en modo wall follower y estamos cerca de la linea original y pero hay diferencia entre tu angulo inciail y final retomamos la linea original
            print("distancia_hacia linea_cen: ",dist,dis_lin)
            if (estado_odom == False) and (dist < 0.01) and (dis_lin > 0.1):
                #print("dis_lin: ",dis_lin)
                estado_odom = True
                #Obtenemos los nuevos puntos iniciales para alinearnos
                self.xin = self.x
                self.yin = self.y
            #Controlamos si seguimos la pared o no
            if estado_odom == False:
                #print("Wall follow")
                v,w = self.follow_left_hand_wall()
            #Publicamos la velocidad
            msg = Twist()
            w = np.clip(w, -self.w_max, self.w_max)
            v = np.clip(v, -self.v_max, self.v_max)
            msg.angular.z = w
            msg.linear.x = v
            self.vel_pub.publish(msg)
            t0 = tf
            self.rate.sleep()
            if self.final == True:
                msg.angular.z = 0
                msg.linear.x = 0
                self.vel_pub.publish(msg)
                break

def range_index(scan, angle):
    #--------------------------------------------------------------
    # Your code here
    #Obtenemos que tan largo es el index del abanico del LIDAR
    num_scans = len(scan.ranges)
    #Obtenemos el angulo maximo y minimo del abanico
    maximo = scan.angle_max
    minimo = scan.angle_min
    #print(maximo)
    #Mapeamos del menor al mayor partiendo el abanico 720 paor (self.rayo_izq < 0.25)or (self.rayo_der < 0.25)rtes
    abanico = np.linspace(minimo,maximo,num_scans)
    #Obtenemos distancia euclidiana del angulo deseado con todo el abanico
    dist = np.sqrt((abanico-angle)**2)
    #obtenemos el index con el que se tuvo una distancia menor para obtener el index del abanico correspondiente
    index = np.argmin(dist)
    return index
    #--------------------------------------------------------------
    #--------------------------------------------------------------    
            


def find_wall_direction(scan):
    #--------------------------------------------------------------
    # Your code here
    #Declaramos el angulo de inclinacion del rayo que va a ser la hipotenusa de nuestro triangulo
    theta = -135
    #Obtenemos la distancia de la hipotenusa de nuestro triangulo
    a = get_distance_in_sector(scan, np.radians(theta+2), np.radians(theta-2))
    #Obtenemos la idstancia perpendicular del robot con el muro
    b = get_distance_in_sector(scan, np.radians(-88), np.radians(-92))
    #Obtenemos el angulo de inclinacion del8,0 robot conforme al muro
    alpha = np.arctan2((-a*np.cos(np.radians(theta))-b),(-a*np.sin(np.radians(theta))))
    return alpha
    #--------------------------------------------------------------
    #--------------------------------------------------------------

    
def get_distance_in_sector(scan, start_angle, end_angle) :
    num_scans = len(scan.ranges)
    #--------------------------------------------------------------
    # Your code here. For documentation of the LaserScan message:
    # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
    #
    # 1) Find the indices into scan.ranges corresponding to the start_ange and end_angle
    # 2) Compute the average range in the sector using np.mean()
    #--------------------------------------------------------------
    #Obtenemos el index del angulo incial
    start_index = range_index(scan,start_angle)
    #Obtenemos el index de angulo final
    end_index = range_index(scan,end_angle)
    #Como en este caso el ultmo index numpy no lo cuenta, en caso de en final estar en el sector negativo lo recorremos para obtener el ultimo valor
    return np.mean(scan.ranges[end_index:start_index])





if __name__ == '__main__':
    puntos = [(8,0),(8,1.5),(0,3),(0,0)]
    rospy.init_node('Follow_right_hand_wall')
    rhw = RightHandRuleController(puntos = puntos)
    rhw.main()
    

