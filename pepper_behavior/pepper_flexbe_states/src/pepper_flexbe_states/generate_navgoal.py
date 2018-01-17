#!/usr/bin/env python

import math
import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped
from tf import transformations


class GenerateNavGoalState(EventState):
    """
    Implements a state that generates a Nav-goal.

    <# x                  float         x-coordinate
    <# y                  float         y-coordinate
    <# theta              float         theta in degrees
    <# frame_id           string        frame_id

    #> generated_goal     PoseStamped   the nav-goal.

    <= done						Indicates completion.
    """

    def __init__(self, x, y, theta=0.0, frame_id='map'):
        """
        Constructor
        """
        super(GenerateNavGoalState, self).__init__(outcomes=['done'], output_keys=['generated_goal'])
        self.x = x
        self.y = y
        self.theta = theta * math.pi / 180
        self.frame_id = frame_id

    def execute(self, userdata):
        """Execute this state"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        quaternion = transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        userdata.generated_goal = pose
        Logger.loginfo('Generated nav-goal: %r' % pose.pose)
        return 'done'

