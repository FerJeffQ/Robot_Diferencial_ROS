#!/usr/bin/env python3


import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from math import pow, atan2, sqrt
from math import radians, degrees
import sys
import tf


class ControlBot:

    def __init__(self):
        # Creamos el nodo y que sea unico "True"
        rospy.init_node('Controlador_de_robot', anonymous=True)

        #Publicar velocidad
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)

        # Suscribirse a la odometria
        self.pose_subscriber = rospy.Subscriber('/odom',
                                                Odometry, self.update_pose)

        self.pose = Pose2D()
        self.rate = rospy.Rate(10)
        

    def update_pose(self, data):
        """Cuando el suscriptor recibe nuevo mensaje se actualiza la posicion."""
        
        position = data.pose.pose.position        
        orientation = data.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        
     
        self.pose.x = round(position.x, 4)
        self.pose.y = round(position.y, 4)
        self.pose.theta = yaw

    def euclidean_distance(self, goal_pose):
        """Distancia euclidiana entre la posicion actual y la meta."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def steering_angle(self, goal_pose):
        """Direccion del angulo."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)



    def angular_vel(self, goal_pose, constant=15):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Mover hacia el objetivo"""
        goal_pose = Pose2D()

        # Get the input from the user.
        #goal_pose.x = float(input("Establece punto x: "))
        #goal_pose.y = float(input("Establece punto y: "))
        point_x, point_y = input("Ingrese las coordenadas separadas por coma  (x,y): ").split(",")
        goal_pose.x = float(point_x)
        goal_pose.y = float(point_y)


        # revisar esta tolerancia, depende de velocidad de control
        distance_tolerance = 0.15

        vel_msg = Twist()
        
      

        while not rospy.is_shutdown() and self.euclidean_distance(goal_pose) >= distance_tolerance:
        
      



              velocidad_linear = self.linear_vel(goal_pose)
              velocidad_angular = self.angular_vel(goal_pose)
              
       

              print("v_lineal: ",velocidad_linear)
              print("v_angular: ",velocidad_angular)                 
              print("pose x: ",self.pose.x)
              print("pose y: ",self.pose.y)
              print("pose obj x: ",goal_pose.x)
              print("pose obj y: ",goal_pose.y)
              print("angulo robot:",self.pose.theta)
              print("angulo objetivo:",self.steering_angle(goal_pose))
              print("distancia euclidiana:",self.euclidean_distance(goal_pose))
              print("--------------------------------------------------------")                 
                      

		# Linear velocity in the x-axis.
              vel_msg.linear.x = velocidad_linear
              vel_msg.linear.y = 0
              vel_msg.linear.z = 0

		# Angular velocity in the z-axis.
              vel_msg.angular.x = 0
              vel_msg.angular.y = 0
              vel_msg.angular.z = velocidad_angular

		# Publishing our vel_msg
              self.velocity_publisher.publish(vel_msg)

		# Publish at the desired rate.
              self.rate.sleep()


        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        
        answer = input("Quieres ingresar otro punto? y/n:  ")
        if answer == "y":
            self.move2goal()
        else:
            rospy.signal_shutdown("Programa finalizado")
            sys.exit()

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = ControlBot()    
        rospy.loginfo("Inicio el node")      
        x.move2goal()
    except KeyboardInterrupt:
        print("Shutting down")      
