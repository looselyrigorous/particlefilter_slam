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
first_odom = 0


odoms = 0
scanners = 0
def odometry(msg):
    global first_odom
    global Particles
    global odoms
    global scanners
    #print('odom:',odoms)




    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    l = msg.pose.pose.orientation.x
    s = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    w, p, th = mp.quaternion_to_radians(w, l, s, z)
    if first_odom == 0:
        Particles = mp.init_particles(x,y,th)
        first_odom +=1
        return
    if odoms%10 != 0:
        odoms += 1
        return
    odoms += 1

    mp.odom_update(Particles, x, y, th)


# rospy.loginfo("w : {} , p: {}, th:{}".format(w,p,th))


def scanner(msg):
    global Particles
    if first_odom == 0:
        return
    global scanners
    #print('scanner',scanners)
    if scanners%10 != 0:
        scanners += 1
        return
    scanners += 1
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_incr = msg.angle_increment
    range_max = msg.range_max
    measurements = np.asarray(msg.ranges)
    mp.map_update(mb.plot_all_lines, mb.prop_map_update,mb.grid_make, pb.map_pos_error_calc,
                  Particles, angle_min, angle_max, angle_incr, measurements, range_max)
    Particles = mp.selectSurvivors(Particles)

# Slam.mapUpdate(Particles,angle_min,angle_max,angle_incr,measurements,range_max)
# Slam.printBestMap(Particles)
# print(measurements)

def main():
    rospy.init_node('my_package')
    #rospy.Subscriber("/kobuki/laser/scan",LaserScan,scanner)

    rospy.Subscriber("/odom",Odometry,odometry)
    rospy.Subscriber("/scan", LaserScan, scanner)
    rospy.spin()


if __name__ == '__main__':
    main()
