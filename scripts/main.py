#!/usr/bin/python
# -*- coding: utf-8 -*-

# [Import]------------------------------->
import math
import sys
import time
import types
import threading
import math # TODO
import numpy as np 
from math import pi
from math import isnan
from datetime import datetime
from matplotlib import pyplot

import tf
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import UInt32MultiArray
from geometry_msgs.msg import Twist

from aruco_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
import rospy
# [ImportScripts]------------------------------->

# [ClassDefine]-------------------------->
class DisplayDisporsalMaster():
    ''' It is DisplayDisporsal task's Master '''
    def __init__(self):
        # ROS TF ------------->>>
        self.listener = tf.TransformListener()

        # BASE PARAM -------------->>>
        self.PARAM_STRAIGHT_BACK_BASIC = 0 # <--- Detect by experiment.
        # 前4,後5.4で微動動作
        self.PARAM_STRAIGHT            = 4.2 # [sec]
        self.PARAM_BACK                = 5.4
        self.PARAM_LONG_STRAIGHT       = 5.3
        self.PARAM_SHORT_STRAIGHT      = 3.6

        self.PARAM_RIGHT_LEFT_BASIC    = 3   # [sec]
        self.PARAM_RIGHT               = 0
        self.PARAM_LEFT                = 0.05

        self.PARAM_BIT_RIGHT           = 1.3 # [sec]
        self.PARAM_BIT_LEFT            = 1.3 # min -> 1.2

        # BASE PARAM -------------->>>
        self.VARID_DEG                 = 30 # [deg]
        self.LIDAR_DEGREE_THRESHOLD    = 4  # [deg]

        # OTHER PARAM ------------>>>
        self.COMMUNICATION_RATE = 15 # <--- AcademicPack communication frequency limit is 20[count/sec].
        self.rate = rospy.Rate(self.COMMUNICATION_RATE)
        self.is_task_complete = False

        # Set rospy to execute a shutdown function when exiting --->>>
        rospy.on_shutdown(self.shutdown)

        # Thread set --------->>>
        #threading.Thread(
        #        target=self.watchListenerLoop,
        #        name="TfListener[Robo ---> Item]",
        #        ).start()
        #threading.Thread(
        #        target=self.watchThreads,
        #        name="ThreadWatcher",
        #        ).start()

        # Othrer init -------->>>
        self.object_point = Point # Point have float x,y,z param
        self.lidar_grad = None
        #self.lidar_dist = None

        # ROS Subscriber ----->>>
        self.markers_sub      = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.markersCB)
        #self.markers_list_sub = rospy.Subscriber('/aruco_marker_publisher/markers_list', UInt32MultiArray, self.markersListCB)
        #self.markers_list_sub = rospy.Subscriber('/aruco_marker_publisher/markers_list', MarkerArray, self.markersListCB)
        #self.object_point_sub = rospy.Subscriber('/point_cloud/object_point', Point, self.pointCB)
        self.arm_move_sub     = rospy.Subscriber('/move_arm/motion_state', Bool, self.receiveArmMotionCB)
        self.lidar_sub        = rospy.Subscriber('/scan', LaserScan, self.lidarCB)

        # ROS Publisher ------>>>
        self.cmd_vel          = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.arm_move_pub     = rospy.Publisher('/move_arm/servo_url', String, queue_size=1)
        #self.move_it_pub      = rospy.Publisher('/', , queue_size=1)


# [CallBack]---------------------------------->
# @param msg sensor_msgs.msg.LaserScan
    def lidarCB(self, msg):
        '''
            正面の障害物の傾き(と距離)をsetする.

            std_msgs/Header header
                uint32 seq
                time stamp
                string frame_id
            float32 angle_min
            float32 angle_max
            float32 angle_increment
            float32 time_increment
            float32 scan_time
            float32 range_min
            float32 range_max
            float32[] ranges <--- important
            float32[] intensities
        '''
        # TODO : 外れ値の除去、デジタルフィルタ
        # TODO : 使うデータをrange_max, range_minから割り出す.
	#	 現状の設定では、hukuyo以外ではうまく動作しないはず.
        if rospy.Time.now().secs == msg.header.stamp.secs:
        #if 1:
            # Lidar's valid angle.
            r      = np.asarray( msg.ranges)[4*(90-self.VARID_DEG):4*(90+self.VARID_DEG)]
            theta  = np.arange( self.VARID_DEG, -1*self.VARID_DEG, -0.25)
            x      = r * np.cos( np.radians( theta ))
            y      = r * np.sin( np.radians( theta ))

            # Least-square method ( y = ax + b ) --->
            x_ave  = np.mean(x)
            y_ave  = np.mean(y)
            x_var  = np.var(x, keepdims=True)
            y_var  = np.var(y, keepdims=True)
            xy_cov = np.cov(x, y)[1][0]
            a      = xy_cov / x_var
            b      = y_ave - a * x_ave
            #print "x_ave ", x_ave
            #print "y_ave ", y_ave
            #print "x_var ", x_var
            #print "y_var ", y_var
            #print "xy_cov", xy_cov
            #print "size_y", y.size
            #print "a", a
            #print "b", b

            # Value set -------------------->>>
            # Transe [gradiate of y=a*x+b] ---> [degree (-180, 180)]
            deg = math.degrees( math.atan2(1, a) )
            if deg > 90:
                #print "!!! turn !!!"
                deg = deg - 180
            #print "deg", deg
            self.lidar_grad = deg
            self.lidar_dist = b

            # Visualization---------------------->>>
            if 0:
                # Origin polar coordinates data visualization--------->
                fig = pyplot.figure()
                pyplot.plot(theta, r)
                pyplot.show()

                # Figure setting ------------------------------------->
                fig = pyplot.figure()
                pyplot.title('2D Lidar')
                pyplot.xlabel('robot-y')
                pyplot.ylabel('robot-x')

                # Least-square method result ( y = ax + b ) ----------> 
                pyplot.plot(x, a*x+b)

                # Origin cartesian coordinates data visualization----->
                ax = fig.add_subplot(1,1,1)
                ax.scatter(x, y)

                pyplot.show()


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
            br2.sendTransform((0.0, 0.0, 1.345),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/robot",
                            "/camera")
            '''
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
            
            print "pos x : ", pos.x
            l = math.sqrt(pow(pos.z, 2) + pow(pos.x, 2))
            deg = math.degrees(math.atan(l)) # tan-1(okuyuki/yoko)
            print deg

            # From camera to item --->
            #print pos
	    '''
            br1 = tf.TransformBroadcaster()
            br1.sendTransform((pos.z, pos.x, -1*pos.y),
                            #tf.transformations.quaternion_from_euler(0, 0, 0),
                            (ori.x, ori.y, ori.z, ori.w),
                            rospy.Time.now(),
                            "/camera",
                            "/item")
                            #str(id_),
	    '''

            # From robot to camera --->
	    '''
            br2 = tf.TransformBroadcaster()
            br2.sendTransform((0.0, 0.0, 1.345),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/robot",
                            "/camera")
	    '''


    def receiveArmMotionCB(self, msg):
        print("arm sub : " + str(msg.data))
        self.is_task_complete = msg.data


# @param msg std_msgs/UInt32MultiArray
    def markersListCB(self, msg):
        #rospy.loginfo("marker len : %s", msg)
        #print type(msg)
        #print msg 
        pass
#            if marker_id == marker.id:
#                pose = marker2mat(marker)
#                k1_list.append(pose)


# @param msg std_msgs/String
    def publishToMotionProgram(self, str_):
        msg = String()
        msg.data = str_
        self.arm_move_pub.publish(msg)
        rospy.sleep(5)
        while self.is_task_complete == False:
            pass
        self.is_task_complete = False

    
# [ClassFunctions]---------------------------->
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist()) # for movement stop
        rospy.sleep(1)


    def watchThreads(self):
        '''
            rosのパッケージやcallbackの仕組みを理解するために見とく
        '''
        #try:
        while not rospy.is_shutdown():
            time.sleep(6)
            tlist = threading.enumerate()
            #if len(tlist) &lt; 2: break
            print "-"*30
            for t in tlist:
                print (t)
            print "-"*30
        #except KeyboardInterrupt:
        #    sys.exit()


    def watchListenerLoop(self):
        while not rospy.is_shutdown():
            try:
                now = rospy.Time(0)
                # Wait.
                self.listener.waitForTransform("/robot", "/item", now, rospy.Duration(3.0))

                # From robot to item.
                (trans,quaternion) = self.listener.lookupTransform('/robot', '/item', now)
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
            print "roll  [rad]: " + str (roll)
            print "pitch [rad]: " + str (pitch)
            print "yaw   [rad]: " + str (yaw)
            print "roll  [deg]: " + str (roll * 360 /(2*3.14))
            print "pitch [deg]: " + str (pitch * 360 /(2*3.14))
            print "yaw   [deg]: " + str (yaw * 360 /(2*3.14))

            # Renew object point.
            self.object_point.x = trans[0]
            self.object_point.y = trans[1]
            self.object_point.z = trans[2]

            #self.rate.sleep()
            time.sleep(1)


    # [Display] ----------------------->>>
    def display(self):
        print "<<< Display >>>"
        rospy.sleep(3)
        try:
            #  --->
            print "Display1"
            #self.publishToMotionProgram("onigiri1.txt")
            #self.publishToMotionProgram("bottle1.txt")
            #self.publishToMotionProgram("obentou.txt")

            # move --->
            self.goBack()
            rospy.sleep(1)

            self.rotateLeft()
            rospy.sleep(1)

            self.goShort()
            rospy.sleep(1)

            self.rotateRight()
            rospy.sleep(1)

            self.adjustBaseAngleFromLidar() # <--- adjust
            rospy.sleep(1)

            self.goStraight()
            rospy.sleep(1)

            # --->
            print "Display2"
            #self.publishToMotionProgram("onigiri2.txt")
            #self.publishToMotionProgram("bottle2.txt")
            #self.publishToMotionProgram("obentou.txt")


        except KeyboardInterrupt:
            sys.exit()

        return 1 # <--- go to moveBase


    # [Disporsal] ----------------------->>>
    def disporsal(self):
        print "<<< DisplayDisporsal >>>"
        return 3 # < --- exit


    # [Move] ---------------------------->>>
    def practice(self):
        print "<<< practice >>>"
        rospy.sleep(3)

        while 1:
            self.adjustBaseAngleFromLidar()
            rospy.sleep(2)
        #self.goLong()
        #rospy.sleep(2)

        #self.rotateRight()
        #rospy.sleep(3)
        #self.rotateRight()
        #rospy.sleep(3)
        #self.rotateRight()
        #rospy.sleep(3)
        #self.rotateRight()
        #rospy.sleep(3)

        #self.rotateLeft()
        #rospy.sleep(3)
        #self.rotateLeft()
        #rospy.sleep(3)
        #self.rotateLeft()
        #rospy.sleep(3)
        #self.rotateLeft()
        #rospy.sleep(3)

        #self.goBack()
        #rospy.sleep(3)

        #self.goLong()
        #rospy.sleep(3)

        #self.goStraight()
        #rospy.sleep(3)

        return -2 # < --- null


    # [Move] ---------------------------->>>
    def adjustBaseAngleFromLidar(self):
        '''
            正面の大きな障害物に対してLIDAR_DEGREE_THRESHOLDの範囲内になるように旋回する.
	    動作が終了したらTrueを返す.

	    指定の閾値よりも現在の障害物に対する角度が小さければ何も動作しないままrerurnする.
	    この場合でもTrueを返す.

            Ridarが動作していないために角度調整が行えない場合はFalseを返す.

            #self.lidar_dist
            self.lidar_grad
        '''
        # TODO タイムアウトを設定する?
        # TODO 近くに壁があるときに
        if self.lidar_grad != None:
            # ロボットが壁に対して正面に立っているか
            while abs( self.lidar_grad ) > self.LIDAR_DEGREE_THRESHOLD:
                if rospy.is_shutdown() == True:
                    sys.exit()
                if self.lidar_grad > 0:
                    # 左手側に壁が近い--->
                    self.rotateBitLeft()
                elif self.lidar_grad < 0:
                    # 右手側に壁が近い--->
                    self.rotateBitRight()
                    rospy.sleep(1.0)
            # 角度合わせ動作終了.
            return True
        else:
            # Ridarが動いていなかった場合.
            return False


    def moveBase(self):
        '''
            move to next task position
            +---------------------+      
            |                     |
            |   |        |    |   |
            |   |        |    |   |
            |   |        |    |   |
            |   +--------+----+   |
            |                     |
            +---------------------+      
        '''
        print "<<< moveBase >>>"
        rospy.sleep(2)

        # next tana
        self.goBack()
        rospy.sleep(1)

        self.rotateLeft()
        rospy.sleep(1)

        self.goLong()
        rospy.sleep(1)

        self.rotateRight()
        rospy.sleep(1)

        self.adjustBaseAngleFromLidar() # adjust
	rospy.sleep(1)

        self.goStraight()
        rospy.sleep(1)

        return 2 # <--- go to disporsal


    def goLong(self):
        print "Long"
        param = self.PARAM_LONG_STRAIGHT
        self.go(param, 1)


    def goShort(self):
        print "Short"
        param = self.PARAM_SHORT_STRAIGHT
        self.go(param, 1)


    def goBack(self):
        print "BACK"
        param = self.PARAM_STRAIGHT_BACK_BASIC + self.PARAM_BACK
        self.go(param, -1)


    def goStraight(self):
        print "Straight"
        param = self.PARAM_STRAIGHT_BACK_BASIC + self.PARAM_STRAIGHT
        self.go(param, 1)


    def go(self, param, f_or_b):
        '''
            Please set param yourself.
        '''
        # Set the forward linear speed [meter/second]
        linear_speed = 0.15

        # Initialize the movement command
        move_cmd = Twist()
        # Set the forward speed
        move_cmd.linear.x = linear_speed * f_or_b

        print "go"

        print move_cmd
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(param)
        
        print "stop"
        # Stop robot.
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


    def rotateBitRight(self):
        print("BitRight")
        param = self.PARAM_BIT_RIGHT
        self.rotate(param, -1, 0.20)


    def rotateBitLeft(self):
        print("BitLeft")
        param = self.PARAM_BIT_LEFT
        self.rotate(param, 1, 0.20)


    def rotateRight(self):
        print("Right")
        param = self.PARAM_RIGHT_LEFT_BASIC + self.PARAM_RIGHT
        self.rotate(param, -1)


    def rotateLeft(self):
        print("Left")
        param = self.PARAM_RIGHT_LEFT_BASIC + self.PARAM_LEFT
        self.rotate(param, 1)


    def rotate(self, param, r_or_l, angular_speed=1.0):
        '''
            Please set param yourself.
        '''
        #param = 1.11 # <--- Detect by experiment.

        # Set the rotation speed [radians/second]
        #angular_speed = 1.0
        # Set the rotation angle [radians]
        # Initialize the movement command
        move_cmd = Twist()
        # Set the forward speed
        #move_cmd.angular.z = angular_speed * num 
        move_cmd.angular.z = angular_speed * r_or_l

        print "go"
        print move_cmd
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(param)
        
        print "stop"
        # Stop robot.
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


# [Main] ----------------------------------------->>>
#if __name__ == '__main__':
rospy.init_node('display_disporsal')


time.sleep(3.0)
node = DisplayDisporsalMaster()
main_state = -2

while not rospy.is_shutdown():
    # Production----->
    if main_state == 0:
        main_state = node.display()
    elif main_state == 1:
        main_state = node.moveBase()
    elif main_state == 2:
        main_state = node.disporsal()

    # Practice------->
    elif main_state == -1:
        main_state = node.practice()
    rospy.sleep(0.1)
            
