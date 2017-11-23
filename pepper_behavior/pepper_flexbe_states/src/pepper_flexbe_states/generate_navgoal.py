#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D


class GenerateNavgoalState(EventState):
    """
    Implements a state that generates a Navgoal.

    <# x        float           x-coordinate
    <# y        float           y-coordinate
    <# theta    float           theta

    #> navgoal  Pose2D		    The navgoal.

    <= done						Indicates completion.
    """

    def __init__(self, x, y, theta):
        """
        Constructor
        """
        super(GenerateNavgoalState, self).__init__(outcomes=['done'], output_keys=['navgoal'])
        self.x = x
        self.y = y
        self.theta = theta

    def execute(self, userdata):
        """Execute this state"""
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta
        userdata.navgoal = pose

        # nothing to check
        return 'done'

    def on_enter(self, userdata):
        # nothing to do
        pass
