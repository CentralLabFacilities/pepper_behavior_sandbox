#!/usr/bin/env python

import math
import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped


class GenerateNavgoalState(EventState):
    """
    Implements a state that generates a Navgoal.

    <# x        float           x-coordinate
    <# y        float           y-coordinate
    <# theta    float           theta
    <# frame_id string          frame_id

    #> navgoal  PoseStamped		    The navgoal.

    <= done						Indicates completion.
    """

    def __init__(self, x, y, theta=0.0, frame_id='map'):
        """
        Constructor
        """
        super(GenerateNavgoalState, self).__init__(outcomes=['done'], output_keys=['navgoal'])
        self.x = x
        self.y = y
        self.theta = theta
        self.frame_id = frame_id

    def execute(self, userdata):
        """Execute this state"""
        pose = PoseStamped()

        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = math.cos(self.theta / 2)
        pose.pose.orientation.x = self.x * math.sin(self.theta / 2)
        pose.pose.orientation.y = self.y * math.sin(self.theta / 2)
        pose.pose.orientation.z = 0.0 * math.sin(self.theta / 2)

        userdata.navgoal = pose

        # nothing to check
        return 'done'

    def on_enter(self, userdata):
        # nothing to do
        pass
