#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger


class WaitState(EventState):
    """
    Implements a state that waits

    <= done						Indicates completion.
    """

    def __init__(self, time=30):
        """
        Constructor
        """
        super(WaitState, self).__init__(outcomes=['done'])

        self._time = time
        self._start_time = None

    def execute(self, userdata):
        """Execute this state"""
        if (rospy.get_rostime()-self._start_time).to_sec() >= self._time:
            return 'done'

    def on_enter(self, userdata):
        self._start_time = rospy.get_rostime()

