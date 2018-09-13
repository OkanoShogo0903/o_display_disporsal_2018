# -*- coding: utf-8 -*-
# [Import]------------------------------->
import rospy
from std_msgs.msg import String
#from std_msgs.msg import Twist
from geometry_msgs.msg import Twist
from math import pi
# [ParameterStart]----------------------->
class master():
    ''' It is DisplayDisporsal task's master '''
    def __init__(self):
        # ROS Subscriber ----->>>
        #self.wheel_state_sub = rospy.Subscriber('google_req/start', String, )

        # ROS Publisher ----->>>
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Parameter set ----->>>
        self.COMMUNICATION_RATE = 15 # <--- AcademicPack communication frequency limit is 20[count/sec].
        self.rate = rospy.Rate(self.COMMUNICATION_RATE)


        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)
    

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist()) # for movement stop
        rospy.sleep(1)


    def main(self):
        #self.straight()
        self.rotate()


    def straight(self):
        # Set the forward linear speed [meter/second]
        linear_speed = 0.2
        # Set the travel distance [meters]
        #goal_distance = 1.0
        goal_distance = 1.0 * 4/5
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


    def rotate(self):
        # Set the rotation speed [radians/second]
        angular_speed = 1.0
        # Set the rotation angle [radians]
        goal_angle = pi * 1.11
        # How long should it take to rotate?
        angular_duration = goal_angle / angular_speed

        # Initialize the movement command
        move_cmd = Twist()
        # Set the forward speed
        move_cmd.angular.z = angular_speed

        # Move forward for a time to go the desired distance
        ticks = int(angular_duration * self.COMMUNICATION_RATE)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()
        
        # Stop robot.
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('display_disporsal')
    m = master()
    m.main()
