#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_actuators import TextToSpeechActuator

"""

"""


class TextToSpeechState(EventState):
    """
    Makes the Robot talk

    -- blocking             bool        Whether or not this state blocks execution
    -- message              String      What to say

    <= done                     Execution succeeded
    <= failed                   Execution failed
    """

    def __init__(self, message, blocking=True):
        """Constructor"""

        super(TextToSpeechState, self).__init__(outcomes=['done', 'failed'])

        self._blocking = blocking
        self._text = message
        self._tts = TextToSpeechActuator()

    def execute(self, userdata):
        if self._tts.is_done():
            Logger.loginfo('returning %s' % self._tts.get_result())
            return self._tts.get_result()

    def on_enter(self, userdata):
        self._tts.say(self._text, self._blocking)

    def on_exit(self, userdata):
        self._tts.cancel()

    def on_stop(self):
        self._tts.cancel()
