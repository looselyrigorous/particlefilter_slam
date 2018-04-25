#!/usr/bin/env python
import os
import sys, traceback
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from Slam import Particle
import Slam


Particles = list()

def odometry(msg):
	x1 = msg.pose.pose.position.x
	y1 = msg.pose.pose.position.y
	x = msg.pose.pose.orientation.x
	y = msg.pose.pose.orientation.y
	z = msg.pose.pose.orientation.z
	w = msg.pose.pose.orientation.w
	w,p,th = Slam.quaternion_to_euler_angle(w, x, y, z)
	

	rospy.loginfo("w : {} , p: {}, th:{}".format(w,p,th))



def scanner(msg):
	angle_min = msg.angle_min
	angle_max = msg.angle_max
	angle_incr = msg.angle_increment
	range_max = msg.range_max
	measurements = msg.ranges
	Slam.mapUpdate(Particles,angle_min,angle_max,angle_incr,measurements,range_max)
	#Slam.printBestMap(Particles)
	#print(measurements)

def main():
	rospy.init_node('my_package')
	global Particles
	Particles = Slam.initParticles()
	#Slam.printBestMap(Particles)
	rospy.Subscriber("/odom",Odometry,odometry)
	rospy.Subscriber("/scan",LaserScan,scanner)
	rospy.spin()

if __name__ == '__main__':
	main()