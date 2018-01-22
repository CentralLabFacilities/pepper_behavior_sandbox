#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from sensor_msgs.msg import LaserScan


class WaitForOpenDoorState(EventState):
    """
    Implements a state that waits for the way to be clear.

    <= done						Indicates completion.
    """

    LASERS_TO_IGNORE = 5
    ERR_THRESHOLD = 0.15
    old_values = None

    def __init__(self):
        """
        Constructor
        """
        super(WaitForOpenDoorState, self).__init__(outcomes=['done'])
        self._topic = '/laser/srd_front/scan'
        self._sub = ProxySubscriberCached({self._topic: LaserScan})

    def execute(self, userdata):
        """Execute this state"""
        while self._sub.has_buffered(self._topic):
            msg = self._sub.get_from_buffer(self._topic)
            values = msg.ranges
            if self.old_values is None:
                self.old_values = values
            else:
                err = 0.0
                for i in range(self.LASERS_TO_IGNORE, len(values)-1-self.LASERS_TO_IGNORE):
                    err += self.old_values[i] - values[i]
                err /= len(values)
                if err < 0 and abs(err) > self.ERR_THRESHOLD:
                    Logger.loginfo('THRESHOLD REACHED! (%.3f) - Door should be open!' % err)
                    return 'done'

                self.old_values = values

    def on_enter(self, userdata):
        self._sub.enable_buffer(self._topic)
        Logger.loginfo('Waiting for open door...')

    def on_stop(self):
        self._sub.unsubscribe_topic(self._topic)

    def on_exit(self):
        self._sub.unsubscribe_topic(self._topic)

