#!/usr/bin/env python

import rospy
import smach

from pepper_behavior.actuators.ros_string_pub import RosStringPub


class AnimationPlayerPepper(smach.State):
    def __init__(self, animation, controller, wait=5):
        self.wait = wait
        self.animation = animation
        smach.State.__init__(self, outcomes=['success'])
        self.pub = controller
        rospy.sleep(1)
    def execute(self, userdata):
        self.pub.publish_animation(self.animation)
        rospy.sleep(self.wait)
        return 'success'
