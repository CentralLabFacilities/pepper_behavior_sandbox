#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from sensor_msgs.msg import LaserScan


class WaitForOpenDoorState(EventState):
    '''
    Implements a state that waits for the way to be clear.

    <= timeout                  Timeout reached
    <= done						Indicates completion.
    '''
    old_values = None

    def __init__(self):
        '''
        Constructor
        '''
        super(WaitForOpenDoorState, self).__init__(outcomes=['done', 'timeout'], )
        self._topic = '/laser/srd_front/scan'
        self._sub = ProxySubscriberCached({self._topic: LaserScan})

    def execute(self, userdata):
        '''Execute this state'''
        if self._sub.has_buffered(self._topic):
            if self._sub.has_buffered(self._topic):
                msg = self._sub.get_from_buffer(self._topic)
                values = msg.ranges
                if self.old_values is None:
                    self.old_values = values
                else:
                    err = 0.0
                    for i in range(0, len(values)-1):
                        err += self.old_values[i] - values[i]
                    err /= len(values)
                    Logger.loginfo('error: %.1f' % err)
                    if err < 0 and abs(err) > 0.15:
                        return 'done'

                    self.old_values = values

    def on_enter(self, userdata):
        self._sub.enable_buffer(self._topic)

        # nothing to do
