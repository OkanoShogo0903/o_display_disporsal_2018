#!/usr/bin/python
# -*- coding: utf-8 -*-

# [Import]------------------------------->
import types
import threading
from math import pi
from datetime import datetime

from std_msgs.msg import String
from std_msgs.msg import UInt32MultiArray
from aruco_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
#from std_msgs.msg import Twist

import tf
import rospy
# [ImportScripts]------------------------------->

# [ClassDefine]-------------------------->
class DisplayDisporsalMaster():
    ''' It is DisplayDisporsal task's Master '''
    def __init__(self):
        # ROS Subscriber ----->>>
        self.markers_sub      = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.markersCB)
        self.markers_list_sub = rospy.Subscriber('/aruco_marker_publisher/markers_list', UInt32MultiArray, self.markersListCB)
        #self.markers_list_sub = rospy.Subscriber('/aruco_marker_publisher/markers_list', MarkerArray, self.markersListCB)
        self.object_point_sub  = rospy.Subscriber('/point_cloud/object_point', Point, self.pointCB)

        # ROS Publisher ------>>>
        self.cmd_vel          = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # ROS TF ------------->>>
        self.listener = tf.TransformListener()

        # Parameter set ------>>>
        self.COMMUNICATION_RATE = 15 # <--- AcademicPack communication frequency limit is 20[count/sec].
        self.rate = rospy.Rate(self.COMMUNICATION_RATE)

        # Set rospy to execute a shutdown function when exiting --->>>
        # rospy.on_shutdown(self.shutdown)

        # Thread set --------->>>
        threading.Thread(
                target=self.watchListenerLoop,
                name="Listener[Robo ---> Item]",
                ).start()

        # Othrer init -------->>>
        self.object_point = Point # Point have float x,y,z param

# [CallBack]---------------------------------->
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

        # From camera to item --->
        br1 = tf.TransformBroadcaster()
        br1.sendTransform((msg.z, msg.x, msg.y),
                        #tf.transformations.quaternion_from_euler(0, 0, 0),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "/item_",
                        "/camera_")

        # From robot to camera --->
        br2 = tf.TransformBroadcaster()
        br2.sendTransform((0.0, 0.0, 1.345),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "/camera_",
                        "/robot_")
        print "pointCB END"


# @param msg aruco_msgs/MarkerArray
    def markersCB(self, msg):
        #rospy.loginfo("==============")
        #print type(msg)
        #print msg 
        for marker in msg.markers:
            #print marker
            #print marker.pose.pose
            pos = marker.pose.pose.position
            ori = marker.pose.pose.orientation
            id_ = marker.id

            # From camera to item --->
            #print pos
            br1 = tf.TransformBroadcaster()
            br1.sendTransform((pos.z, pos.x, -1*pos.y),
                            #tf.transformations.quaternion_from_euler(0, 0, 0),
                            (ori.x, ori.y, ori.z, ori.w),
                            rospy.Time.now(),
                            "/item_",
                            "/camera_")
                            #str(id_),

            # From robot to camera --->
            br2 = tf.TransformBroadcaster()
            br2.sendTransform((0.0, 0.0, 1.345),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/camera_",
                            "/robot_")


# @param msg std_msgs/UInt32MultiArray
    def markersListCB(self, msg):
        #rospy.loginfo("marker len : %s", msg)
        #print type(msg)
        #print msg 
        pass
#            if marker_id == marker.id:
#                pose = marker2mat(marker)
#                k1_list.append(pose)


# [ClassFunctions]---------------------------->
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist()) # for movement stop
        rospy.sleep(1)


    def watchListenerLoop(self):
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                # waitForTransform(frame(from), frame(to), time, timeout)
                #listener.waitForTransform("/robot", "/item", now, rospy.Duration(3.0))

                # From robot to item.
                (trans,quaternion) = self.listener.lookupTransform('/robot_', '/item_', now)
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                #print "*"*50
                continue
            rospy.loginfo("********************")

            # Return Euler angles from quaternion for specified axis sequence.
            euler = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3])),
            roll  = euler[0][0]
            pitch = euler[0][1]
            yaw   = euler[0][2]

            print "trans : " + str (trans)
            print "roll  : " + str (roll)
            print "pitch : " + str (pitch)
            print "yaw   : " + str (yaw)

            # Renew object point.
            self.object_point.x = x
            self.object_point.y = y
            self.object_point.z = z

            self.rate.sleep()


    def move(self):
        rospy.sleep(5)

        self.rotateRight()
        rospy.sleep(10)

        self.goStraight()
        rospy.sleep(10)

        self.rotateLeft()
        rospy.sleep(10)


    def goStraight(self):
        '''
            Please set param yourself.
        '''

        param = 4/5 # <--- Detect by experiment.
        # Set the forward linear speed [meter/second]
        linear_speed = 0.2
        # Set the travel distance [meters]
        #goal_distance = 1.0
        goal_distance = 1.0 * param
        # How long should it take us to get there?
        linear_duration = goal_distance / linear_speed

        # Initialize the movement command
        move_cmd = Twist()
        # Set the forward speed
        move_cmd.linear.x = linear_speed

        # Move forward for a time to go the desired distance
        ticks = int(linear_duration * self.COMMUNICATION_RATE)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()
        
        # Stop robot.
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


    def rotateRight(self):
        self.rotate(1)


    def rotateLeft(self):
        self.rotate(-1)


    def rotate(self, num):
        '''
            Please set param yourself.
        '''
        param = 1.11 # <--- Detect by experiment.

        # Set the rotation speed [radians/second]
        angular_speed = 1.0
        # Set the rotation angle [radians]
        goal_angle = pi * param
        # How long should it take to rotate?
        angular_duration = goal_angle / angular_speed

        # Initialize the movement command
        move_cmd = Twist()
        # Set the forward speed
        move_cmd.angular.z = angular_speed * num

        # Move forward for a time to go the desired distance
        ticks = int(angular_duration * self.COMMUNICATION_RATE)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()
        
        # Stop robot.
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


#if __name__ == '__main__':
rospy.init_node('display_disporsal')
node = DisplayDisporsalMaster()

#rate = rospy.Rate(10.0)
node.move()

rospy.spin()
