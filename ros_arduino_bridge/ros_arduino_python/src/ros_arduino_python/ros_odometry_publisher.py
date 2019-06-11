#!/usr/bin/env python

# Programa en el que se implementa un nodo que publica un mensaje con la odometria
# del robot. Para ello se incorpora un socket UDP que recibe la pose

# Librerias

# Funciones matematicas
import math 
from math import sin, cos, pi 

# Librerias para la publicacion de la odometria
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import socket # Para poder utilizar los sockets

rospy.init_node('odometry_publisher') # Se inicia el nuevo nodo

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50) # Se publicara en el topico odom
odom_broadcaster = tf.TransformBroadcaster()

# Variables necesarias para los calculos posteriores
x = 0.0
y = 0.0
th = math.pi/2

last_x = 0.0
last_y = 0.0
last_th = math.pi/2

vx = 0.0
vy = 0.0
vth = 0.0

# Se llevara un control del tiempo para calcular la velocidad del robot
current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(1.0)

# Parametros para establecer la conexion
UDP_IP = "0.0.0.0"
UDP_PORT = 4446

# Se inicia el socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while not rospy.is_shutdown():
    current_time = rospy.Time.now() # Se almacena el tiempo actual
    
    # Se espera a recibir el mensaje del Arduino
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    # En data se encuentra el string "(x,y,Theta) 
    
    # Tras recibir el mensaje se actualiza la pose antes de publicarla
    
    stringX = ""
    stringY = ""
    stringTh = ""

    i = 0 # iteracion
    j = 0 # string que toca rellenar
    while data[i] != ")" and i < 100:
      
      if (data[i] == "(" or data[i] == ","):
        # Se cambia de string a rellenar
        j = j + 1
      else:
        if j == 1:
          stringX = stringX + data[i]
        if j == 2:
          stringY = stringY + data[i]
        if j == 3:
          stringTh = stringTh + data[i]

      i = i + 1
    
    # Se convierten los valores a float para poder trabajar con ellos
    
    x = float(stringX)/100 
    y = float(stringY)/100
    th = float(stringTh)*math.pi/180
    
    # Se prepara el mensaje con la odometria del robot
    dt = (current_time - last_time).to_sec() # Se calcula la variacion de tiempo en la ultima ejecucion
    
    
    # Se averigua cuanto ha cambiado la pose en el ultimo ciclo
    delta_x = x - last_x
    delta_y = y - last_y
    delta_th = th - last_th 
    
    # Se calcula la velocidad como el espacio que se ha recorrido entre el tiempo que se ha tardado
    
    vx = delta_x/dt
    vy = delta_y/dt
    vth = delta_th/dt
    
    # Al ser la odometria de 6 grados de libertad hay que crear el cuaternion
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # Primero se publica la transformada del paso anterior
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # Se publica el mensaje con la odometria
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # Se situa el robot en el plano
    odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))

    # Se indica la velocidad del robot
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # Finalmente se publica el mensaje
    odom_pub.publish(odom)

    # Se almacenan los valores de la pose en este momento para conocerlos en la siguiente ejecucion
    last_x = x
    last_y = y
    last_th = th

    last_time = current_time # Se almacena el momento en el que se termina este ciclo

