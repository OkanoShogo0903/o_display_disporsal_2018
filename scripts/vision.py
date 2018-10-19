#!/usr/bin/python
# -*- coding: utf-8 -*-

# [Import]------------------------------->
import sys
import json
import math
import time
import types
import threading
import numpy as np 
import math
from math import pi
from math import isnan
from datetime import datetime
from matplotlib import pyplot

import tf
import rospy
from std_msgs.msg import String, Bool, UInt32MultiArray
from geometry_msgs.msg import Twist , Point , Twist
from aruco_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan

# [Import]------------------------------->
class DisplayDisposalVision():
    def __init__(self):
        # ROS TF ------------->>>
        self.listener = tf.TransformListener()

        # ROS Subscriber ----->>>
        self.markers_sub      = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.markersCB)
        #self.markers_list_sub = rospy.Subscriber('/aruco_marker_publisher/markers_list', MarkerArray, self.markersListCB)
        #self.object_point_sub = rospy.Subscriber('/point_cloud/object_point', Point, self.pointCB)

    # @param msg std_msgs/UInt32MultiArray
    def markersListCB(self, msg):
        pass


    # @param msg aruco_msgs/MarkerArray
    def markersCB(self, msg):
        '''
        Camera Coodinate.
        Camera locate in (x,y,z)=(0,0,0)
        +---------------------+
        |         y^  z^      |
        |          |  /       |
        |          | /        |
        |          |/         |
        |   <------+-------   |
        |          |      x   |
        |          |          |
        |          |          |
        |                     |
        +---------------------+
        '''
        for marker in msg.markers:
            #print marker
            #print marker.pose.pose
            pos = marker.pose.pose.position
            ori = marker.pose.pose.orientation
            id_ = marker.id
            
            # From camera to item --->
            print "camera coordinate :", pos
            #print tf.transformations.quaternion_from_euler(0, -45, 0)
            #print tf.transformations.euler_from_quaternion(tf.transformations.quaternion_from_euler(0, 315, 0))
            #print tf.transformations.euler_from_quaternion(tf.transformations.quaternion_from_euler(0, -45, 0))
            br1 = tf.TransformBroadcaster()
            br1.sendTransform((pos.z, pos.x, pos.y),
                            (ori.x, ori.y, ori.z, ori.w),
                            #tf.transformations.quaternion_from_euler(0, 45, 0),
                            #(0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/item"+str(marker.id),
                            "/camera")
                            #str(id_),
            # From robot to camera --->
            br2 = tf.TransformBroadcaster()
            br2.sendTransform((-0.040, 0.0, 0.277),
                            tf.transformations.quaternion_from_euler(0, 45, 0),
                            #(0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/camera",
                            "/robot")


    # @param msg geometry_msgs.msg.Point
    def pointCB(self, msg):
        '''
        # Camera Coodinate.
        # Camera locate in (x,y,z)=(0,0,0)
        #+---------------------+
        #|        y ^   ^      |
        #|          |  /  z    |
        #|          | /        |
        #|          |/         |
        #|   -------+------> x |
        #|          |          |
        #|          |          |
        #|          |          |
        #|                     |
        #+---------------------+
        '''
        print "pointCB ----->>>"
        print msg.x, msg.y, msg.z
        print msg.z, msg.x, msg.y + 1.345
        #tf.transformations.quaternion_from_euler(0, 0, 0),
        
        if isnan(msg.x) or isnan(msg.y) or isnan(msg.z):
            return
        else:
            # From camera to item --->
            '''
            br1 = tf.TransformBroadcaster()
            br1.sendTransform((msg.z, msg.x, msg.y),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/camera",
                            "/item")
            '''

            # From robot to camera --->
            '''
            br2 = tf.TransformBroadcaster()
            br2.sendTransform((-0.040, 0.0, 0.277),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/robot",
                            "/camera")
            '''
            print "pointCB END"



# [Main] ----------------------------------------->>>
#if __name__ == '__main__':
rospy.init_node('dd_vision')

time.sleep(3.0)
node = DisplayDisposalVision()
rospy.spin()
