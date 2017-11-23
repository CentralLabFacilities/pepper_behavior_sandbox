#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String


class WaitForNaoQiSpeechState(EventState):
    """
    Implements a state that waits for a NaoQI-Speech-Command.

    -- string_to_rec            Speech Command to Listen for

    <= done						Indicates completion.
    """

    recognized = False

    def __init__(self, string_to_rec, topic='/pepper_robot/speechrec/context'):
        """
        Constructor
        """
        super(WaitForNaoQiSpeechState, self).__init__(outcomes=['done'])
        self._topic = topic
        self._target_string = string_to_rec
        self._sub = ProxySubscriberCached()

    def _speech_callback(self, msg):
        if msg.data.lower() == self._target_string:
            self.recognized = True

    def execute(self, userdata):
        """Execute this state"""
        if self.recognized:
            return 'done'

    def on_enter(self, userdata):
        self._sub.subscribe(self._topic, String, self._speech_callback)
