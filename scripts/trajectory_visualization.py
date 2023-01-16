#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import time


def callback(data):
        global xAnt
        global yAnt
        global cont

    #Is created the pose msg, its necessary do it each time because Python manages objects by reference, 
        #and does not make deep copies unless explicitly asked to do so.
        pose = PoseStamped()    

    #Set a atributes of the msg
        pose.header.frame_id = "odom"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    #To avoid repeating the values, it is found that the received values are differents
        if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
                #Set a atributes of the msg
                pose.header.seq = path.header.seq + 1
                path.header.frame_id="odom"
                path.header.stamp=rospy.Time.now()
                pose.header.stamp = path.header.stamp
                path.poses.append(pose)
                #Published the msg


        pub.publish(path)

    #Save the last position
        xAnt=pose.pose.orientation.x
        yAnt=pose.pose.position.y
        return path




if __name__ == '__main__':

        #Variable initialization
        global xAnt
        global yAnt
        global cont
        xAnt=0.0
        yAnt=0.0


        #Node and msg initialization
        rospy.init_node('path_odom_plotter')
        nodename = rospy.get_name()
        rospy.loginfo("Nodo %s Iniciado" % nodename)
        pub = rospy.Publisher('/odompath', Path, queue_size=10)
        path = Path() #creamos el mensaje path de tipo path 
        msg = Odometry()
        #Subscription to the topic
        msg = rospy.Subscriber('/odom', Odometry, callback) 
        rate = rospy.Rate(30) # 30hz

try:
	while not rospy.is_shutdown():
        	#rospy.spin()
        	rate.sleep()
except rospy.ROSInterruptException:

	pass
