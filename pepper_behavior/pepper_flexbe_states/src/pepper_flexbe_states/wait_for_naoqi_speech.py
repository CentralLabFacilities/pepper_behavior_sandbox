#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String


class WaitForNaoQiSpeechState(EventState):
    """
    Implements a state that waits for a NaoQI-Speech-Command.

    -- strings_to_rec        string[] 			List of speech commands to listen for

    <= done						Indicates completion.
    """

    recognized = None

    def __init__(self, strings_to_rec, topic='/pepper_robot/speechrec/context'):
        """
        Constructor
        """
        super(WaitForNaoQiSpeechState, self).__init__(outcomes=strings_to_rec)
        self._topic = topic
        self._target_strings = strings_to_rec
        self._sub = ProxySubscriberCached()

    def _speech_callback(self, msg):
	for result in self._target_strings:
            if msg.data.lower() == result:
                self.recognized = result

    def execute(self, userdata):
        """Execute this state"""
        if self.recognized is not None:
            return self.recognized

    def on_enter(self, userdata):
        self._sub.subscribe(self._topic, String, self._speech_callback)
