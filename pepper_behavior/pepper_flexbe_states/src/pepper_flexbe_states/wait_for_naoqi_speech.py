#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String


class WaitForNaoQiSpeechState(EventState):
    """
    Implements a state that waits for a NaoQI-Speech-Command.

    -- strings_to_rec        string[] 			List of speech commands to listen for
    -- outcomes              string[] 			List of outcomes corresponding to the speech commands

    <= done						Indicates completion.
    """

    recognized = None
    _outcomes = None

    def __init__(self, strings_to_rec, outcomes, topic='/pepper_robot/speechrec/context'):
        """
        Constructor
        """
        super(WaitForNaoQiSpeechState, self).__init__(outcomes=outcomes)
        self._topic = topic
        self._outcomes = outcomes
        self._target_strings = strings_to_rec
        self._sub = ProxySubscriberCached()

    def _speech_callback(self, msg):
        Logger.loginfo('speechrec callback:%s' % msg.data.lower())
        for result in self._target_strings:
            Logger.loginfo('\tspeechrec check:%s' % result.lower())
            if msg.data.lower() == result.lower():
                Logger.loginfo('\t\tspeechrec MATCH!')
                self.recognized = result.lower()

    def execute(self, userdata):
        """Execute this state"""
        if self.recognized is not None:
            return self.outcomes[self._target_strings.index(self.recognized)]

    def on_enter(self, userdata):
        self._sub.subscribe(self._topic, String, self._speech_callback)
