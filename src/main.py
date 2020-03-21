#!/usr/bin/env python
import rospy
import sys

import numpy as np
import random
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

        rospy.init_node('usi_assignment1', anonymous=True)

        rospy.wait_for_service('clear')
        self.clear = rospy.ServiceProxy('clear', std_srvs.srv.Empty())

        rospy.wait_for_service('kill')
        self.turtle_kill = rospy.ServiceProxy('kill', turtlesim.srv.Kill)

        rospy.wait_for_service('reset')
        self.reset = rospy.ServiceProxy('reset', std_srvs.srv.Empty())
        self.reset()

        rospy.wait_for_service('spawn')
        self.spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        self.spawn(random.randint(1,10), random.randint(1,10), deg2rad(random.randint(1,360)), 'turtle_writer')

        rospy.wait_for_service('turtle1/set_pen')
        self.offender_set_pen = rospy.ServiceProxy('turtle1/set_pen', turtlesim.srv.SetPen)
        self.offender_set_pen(0,0,0,0,1) # default function

        rospy.wait_for_service('turtle_writer/set_pen')
        self.writer_set_pen = rospy.ServiceProxy('turtle_writer/set_pen', turtlesim.srv.SetPen)
        self.writer_set_pen_off(0) # custom function


        # Publish to the topic '/turtleX/cmd_vel'
        self.velocity_publisher = rospy.Publisher('/turtle_writer/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/turtleX/pose'
        self.writer_pose_subscriber = rospy.Subscriber('/turtle_writer/pose', Pose, self.writer_pose_callback)
        self.offender_pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.offender_pose_callback)
        

        self.pose = Pose()
        self.pose_offender = Pose()
        self.rate = rospy.Rate(10)
        self.state = None



    def writer_pose_callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def offender_pose_callback(self, data):
        self.pose_offender = data
        self.pose_offender.x = round(self.pose_offender.x, 4)
        self.pose_offender.y = round(self.pose_offender.y, 4)


    def writer_set_pen_off(self, off):
        self.writer_set_pen(200,200,200,2,off)


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


    def turtle_move2goal(self, x, y, angular_constant=6, linear_constant=1.5):
        goal_pose = Pose()
        goal_pose.x = x
        goal_pose.y = y
        distance_tolerance = 1
        vel_msg = Twist()

        while not rospy.is_shutdown() and self.euclidean_distance(goal_pose) >= distance_tolerance:

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

        self.turtle_stop()


    def turtle_rotate(self, angle, clockwise=False, speed=100):

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

        while not rospy.is_shutdown() and current_angle < relative_angle:
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        self.turtle_stop()
        

    def turtle_stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()




if __name__ == '__main__':

    usi = UsiAngryTurtle()

    writing_poses = [
        (-2, -2), (1, 11), (1, 8), # starting position
        (-1, -1), (1, 3), (2, 2), (3, 4), (3, 10), # letter U
        (-2, -2), (9,10), (6, 9), # repositioning
        (-1, -1), (5, 9), (4, 7), (5, 5.5), (6, 5.5), (7, 4), (6, 2.5), (3, 2.5), # letter S
        (-2, -2), (9, 2), (9, 3.5), # repositioning
        (-1, -1), (9, 6), (9, 10) # letter I
    ]

    while(not rospy.is_shutdown()): # infinite loop

        usi.state = State.WRITING                                           ####### Change Status: WRITING
        rospy.loginfo(usi.state)

        for index in range(len(writing_poses)): # writes USI over and over again

            x = writing_poses[index][0]
            y = writing_poses[index][1]

            if x == -2:
                usi.writer_set_pen_off(1)   # disable pen

            elif x == -1:
                usi.writer_set_pen_off(0)   # enable pen


            else:                           # normal behaviour
                vel_msg = Twist()
                goal_pose = Pose()
                goal_pose.x = x
                goal_pose.y = y

                while not rospy.is_shutdown() and usi.euclidean_distance(goal_pose) >= 1: # move to goal
                    vel_msg.linear.x = usi.linear_vel(goal_pose, 1.5)
                    vel_msg.angular.z = usi.angular_vel(goal_pose, 6)
                    usi.velocity_publisher.publish(vel_msg)
                    usi.rate.sleep()

                    # Intercepting the offender
                    if(usi.euclidean_distance(usi.pose_offender) < 2):
                        usi.state = State.ANGRY                             ####### Change Status: ANGRY
                        rospy.loginfo(usi.state)
                        usi.writer_set_pen_off(0)

                        # Following the offender
                        while not rospy.is_shutdown() and usi.euclidean_distance(usi.pose_offender) >= 0.3:
                            vel_msg.linear.x = usi.linear_vel(usi.pose_offender, 1.5)
                            vel_msg.angular.z = usi.angular_vel(usi.pose_offender, 6)
                            usi.velocity_publisher.publish(vel_msg)
                            usi.rate.sleep()

                        usi.turtle_stop()
                        usi.turtle_kill('turtle1') # offender has been caught

                        # NOTE: `rostopic echo /turtle1/pose` has 3 to 8 seconds delay after respawning
                        # we update this coords in order to achieve a sort of bugfix (preventing offender to be detected where it is not)
                        # anyway, this way we still prevent offender to be detected correctly during the first seconds of delay
                        usi.pose_offender.x = -100 
                        usi.pose_offender.y = -100

                        usi.state = State.RETURNING                         ####### Change Status: RETURNING
                        rospy.loginfo(usi.state)
                        break


            # Returing to proper position
            if usi.state == State.RETURNING:
                usi.turtle_move2goal(writing_poses[1][0], writing_poses[1][1])
                usi.clear()
                usi.spawn(random.randint(1,10), random.randint(1,10), deg2rad(random.randint(1,360)), 'turtle1')
                usi.offender_set_pen(0,0,0,0,1)
                usi.rate.sleep()
                break

    
    rospy.spin()