#!/usr/bin/env python
import rospy
import sys

import numpy as np
from math import pow,atan2,sqrt
from enum import Enum

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import turtlesim.srv
import std_srvs.srv



class State(Enum):
    WRITING = 1
    ANGRY = 2
    RETURNING = 3


def deg2rad(degrees):
	PI = 3.1415926535897
	return degrees * 2 * PI / 360


class UsiAngryTurtle:

    def __init__(self, linear=0.2, angular=0.0):
        """init"""
        rospy.init_node('usi_assignment1', anonymous=True)

        # reset turtlesim
        rospy.wait_for_service('reset')
    	resetter = rospy.ServiceProxy('reset', std_srvs.srv.Empty())
    	resetter()

        # spawning new writer turtle
        rospy.wait_for_service('spawn')
    	spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    	spawner(1, 9, deg2rad(270), 'turtle_writer')

        # reset turtlesim
        rospy.wait_for_service('turtle_writer/set_pen')
    	self.writer_set_pen = rospy.ServiceProxy('turtle_writer/set_pen', turtlesim.srv.SetPen)
    	self.writer_set_pen_off(0)

        # Publish to the topic '/turtleX/cmd_vel'
        self.velocity_publisher = rospy.Publisher('/turtle_writer/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtleX/pose'
        self.pose_subscriber = rospy.Subscriber('/turtle_writer/pose',
                                                Pose, self.turtlesim_pose_callback)
        
    	self.state = State.WRITING
        self.set_linear = linear
        self.set_angular = angular

        self.pose = Pose()
        self.rate = rospy.Rate(10)


    def writer_set_pen_off(self, off):
    	self.writer_set_pen(200,200,200,2,off)


    def turtlesim_pose_callback(self, data):
        """A new turltesim Pose has arrived. See turtlesim Pose msg definition."""
        
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        # rospy.loginfo("Pose received (%.5f, %.5f, %.5f) (%.5f, %.5f)" % (data.x, data.y, data.theta, data.linear_velocity, data.angular_velocity))


    def rotate(self, angle, clockwise=False, speed=100):

	    vel_msg = Twist()

	    # Converting from angles to radians
	    relative_angle = deg2rad(angle)
	    angular_speed = deg2rad(speed)

	    # We wont use linear components
	    vel_msg.linear.x=0
	    vel_msg.linear.y=0
	    vel_msg.linear.z=0
	    vel_msg.angular.x = 0
	    vel_msg.angular.y = 0

	    # Checking if our movement is CW or CCW
	    if clockwise:
	        vel_msg.angular.z = -abs(angular_speed)
	    else:
	        vel_msg.angular.z = abs(angular_speed)

	    # Setting the current time for distance calculus
	    t0 = rospy.Time.now().to_sec()
	    current_angle = 0

	    while current_angle < relative_angle and not rospy.is_shutdown():
	        self.velocity_publisher.publish(vel_msg)
	        t1 = rospy.Time.now().to_sec()
	        current_angle = angular_speed*(t1-t0)

	    # Forcing our robot to stop
	    vel_msg.angular.z = 0
	    self.velocity_publisher.publish(vel_msg)


    def linear_vel(self, goal_pose, constant):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))


    def move2goal(self, x, y, angular_constant=6, linear_constant=1.5):
        goal_pose = Pose()
        goal_pose.x = x
        goal_pose.y = y
        distance_tolerance = 1
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance and not rospy.is_shutdown():

            # Porportional Controller
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis:
            vel_msg.linear.x = self.linear_vel(goal_pose, linear_constant)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose, angular_constant)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


    def write(self):
    	# U
	    usi.move2goal(1, 9)
	    usi.move2goal(1, 4)
	    usi.move2goal(2, 3, 4)
	    usi.move2goal(3, 5, 4)
	    usi.move2goal(3, 10)
	    # S
	    usi.rotate(90, True)
	    usi.writer_set_pen_off(1)
	    usi.move2goal(7, 10)
	    usi.rotate(180, True)
	    usi.writer_set_pen_off(0)
	    usi.move2goal(5, 9)
	    usi.move2goal(4, 7.5)
	    usi.move2goal(5, 6)
	    usi.move2goal(6, 6)
	    usi.move2goal(7, 4.5)
	    usi.move2goal(6, 3)
	    usi.move2goal(3, 3)
	    # I
	    usi.rotate(180, True)
	    usi.writer_set_pen_off(1)
	    usi.move2goal(9, 3)
	    usi.rotate(90, False)
	    usi.writer_set_pen_off(0)
	    usi.move2goal(9, 10)


    def basic_move(self):
        """Moves the turtle"""
        vel_msg = Twist()
        vel_msg.linear.x = self.set_linear #0.2 # m/s
        vel_msg.angular.z = self.set_angular #0.0 # rad/s

        # single message
        #self.velocity_publisher.publish(vel_msg)
        
        # several messages at a rate
        while not rospy.is_shutdown():
            # Publishing vel_msg
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            self.rate.sleep()

        # waiting until shutdown flag (e.g. ctrl+c)
        rospy.spin()




if __name__ == '__main__':
    param_linear = 0.5
    param_angular = 0.0

    if len(sys.argv) == 3:
        param_linear = sys.argv[1]
        param_angular = sys.argv[2]
        print "Requested linear and angular velocities: %f, %f" % (param_linear, param_angular)
    else:
        print "Not all parameters were sent, working with default linear and angular velocities"

    usi = UsiAngryTurtle(param_linear, param_angular)
    usi.write()
    
    rospy.spin()


