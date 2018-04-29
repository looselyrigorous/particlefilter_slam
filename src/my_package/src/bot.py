#!/usr/bin/env python
import os
import sys, traceback
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from Slam import Particle
import Slam
import MapBuilder as mb
import ModParticle as mp
import numpy as np
import Propabilities as pb
Particles = list()


def odometry(msg):
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    w, p, th = mp.quaternion_to_radians(w, x, y, z)
    mp.odom_update(Particles, x1, y1, th)


# rospy.loginfo("w : {} , p: {}, th:{}".format(w,p,th))


def scanner(msg):
    global Particles
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_incr = msg.angle_increment
    range_max = msg.range_max
    measurements = np.asarray(msg.ranges)
    mp.map_update(mb.plot_all_lines, mb.prop_map_update,mb.grid_make, pb.map_radius_error_calc,
                  Particles, angle_min, angle_max, angle_incr, measurements, range_max)
    Particles = mp.selectSurvivors(Particles)

# Slam.mapUpdate(Particles,angle_min,angle_max,angle_incr,measurements,range_max)
# Slam.printBestMap(Particles)
# print(measurements)

def main():
    rospy.init_node('my_package')
    global Particles
    Particles = mp.init_particles()
    #rospy.Subscriber("/kobuki/laser/scan",LaserScan,scanner)

    rospy.Subscriber("/odom",Odometry,odometry)

    rospy.Subscriber("/scan", LaserScan, scanner)
    rospy.spin()


if __name__ == '__main__':
    main()
