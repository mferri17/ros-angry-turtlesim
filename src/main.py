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


PI = 3.1415926535897

def deg2rad(degrees):
    return degrees * 2 * PI / 360

def rad2deg(rad):
    return rad * 360 / PI / 2


class UsiAngryTurtle:

    def __init__(self, linear=0.2, angular=0.0):
        """init"""
        rospy.init_node('usi_assignment1', anonymous=True)

        # reset turtlesim
        rospy.wait_for_service('reset')
        resetter = rospy.ServiceProxy('reset', std_srvs.srv.Empty())
        resetter()

        # spawning new writer turtle
        self.init_x = 1
        self.init_y = 9
        self.init_theta = deg2rad(270)
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        spawner(self.init_x, self.init_y, self.init_theta, 'turtle_writer')

        # reset turtlesim
        rospy.wait_for_service('turtle_writer/set_pen')
        self.writer_set_pen = rospy.ServiceProxy('turtle_writer/set_pen', turtlesim.srv.SetPen)
        self.writer_set_pen_off(0)

        # Publish to the topic '/turtleX/cmd_vel'
        self.velocity_publisher = rospy.Publisher('/turtle_writer/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtleX/pose'
        self.pose_subscriber = rospy.Subscriber('/turtle_writer/pose',
                                                Pose, self.writer_pose_callback)
        
        self.pose = Pose()
        self.rate = rospy.Rate(10)


    def writer_set_pen_off(self, off):
        self.writer_set_pen(200,200,200,2,off)


    def writer_pose_callback(self, data):
        """A new turltesim Pose has arrived. See turtlesim Pose msg definition."""
        
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

        # TODO tmp
        if self.state == State.WRITING and self.pose.x > 3:
            self.turtle_start_return()

        # rospy.loginfo("Pose received (%.5f, %.5f, %.5f) (%.5f, %.5f)" % (data.x, data.y, data.theta, data.linear_velocity, data.angular_velocity))


    def correct_state(self, state):
        return not rospy.is_shutdown() and (state == None or state == self.state)


    def linear_vel(self, goal_pose, constant):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angle_difference(self, angle1, angle2):
    	return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))

    def angular_vel(self, goal_pose, constant):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.angle_difference(self.steering_angle(goal_pose), self.pose.theta)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))


    def turtle_move2goal(self, state, x, y, angular_constant=6, linear_constant=1.5):
        goal_pose = Pose()
        goal_pose.x = x
        goal_pose.y = y
        distance_tolerance = 1
        vel_msg = Twist()

        rospy.loginfo(state)

        while self.correct_state(state) and self.euclidean_distance(goal_pose) >= distance_tolerance:
            rospy.loginfo("%s %s" % (goal_pose.x, goal_pose.y))

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
        # self.turtle_stop()


    def turtle_rotate(self, state, angle, clockwise=False, speed=100):

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

        while self.correct_state(state) and current_angle < relative_angle:
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        # Forcing our robot to stop
        # self.turtle_stop()
        

    def turtle_stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()


    def turtle_start_write(self):
        rospy.loginfo('Start WRITING')
        self.state = State.WRITING
        self.turtle_stop()
        # U
        self.turtle_move2goal(State.WRITING, 1, 9)
        self.turtle_move2goal(State.WRITING, 1, 4)
        self.turtle_move2goal(State.WRITING, 2, 3, 4)
        self.turtle_move2goal(State.WRITING, 3, 5, 4)
        self.turtle_move2goal(State.WRITING, 3, 10)
        # repositioning
        self.writer_set_pen_off(1)
        self.turtle_rotate(State.WRITING, 90, True)
        self.turtle_move2goal(State.WRITING, 7, 9)
        self.turtle_rotate(State.WRITING, 180, True)
        self.writer_set_pen_off(0)
        # S
        self.turtle_move2goal(State.WRITING, 5, 9)
        self.turtle_move2goal(State.WRITING, 4, 7.5)
        self.turtle_move2goal(State.WRITING, 5, 6)
        self.turtle_move2goal(State.WRITING, 6, 6)
        self.turtle_move2goal(State.WRITING, 7, 4.5)
        self.turtle_move2goal(State.WRITING, 6, 3)
        self.turtle_move2goal(State.WRITING, 3, 3)
        # repositioning
        self.writer_set_pen_off(1)
        self.turtle_rotate(State.WRITING, 180, True)
        self.turtle_move2goal(State.WRITING, 9, 3)
        self.turtle_rotate(State.WRITING, 90, False)
        self.writer_set_pen_off(0)
        # I
        self.turtle_move2goal(State.WRITING, 9, 10)
        self.turtle_stop()


    def turtle_start_return(self):
        rospy.loginfo('Start RETURNING')
        self.state = State.RETURNING
        self.turtle_stop()
        self.rate.sleep()
        # rospy.loginfo(rad2deg(self.init_theta))
        # rospy.loginfo(rad2deg(self.pose.theta))
        # rospy.loginfo(rad2deg(self.init_theta) - rad2deg(self.pose.theta))
        # self.turtle_rotate(State.RETURNING, 120)
        self.turtle_move2goal(State.RETURNING, self.init_x, self.init_y)
        # self.turtle_rotate(State.RETURNING, rad2deg(self.init_theta) - rad2deg(self.pose.theta))
        self.turtle_stop()




if __name__ == '__main__':

    usi = UsiAngryTurtle()

    usi.turtle_start_write()
    
    rospy.spin()


