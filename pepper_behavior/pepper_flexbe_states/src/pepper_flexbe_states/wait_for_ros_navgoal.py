#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseStamped


class WaitForRosNavgoalState(EventState):
    """
    Implements a state that waits for a Navgoal on a RosTopic.

    <# topic    string              The ROS topic to listen on

    #> navgoal  PoseStamped		    The navgoal.

    <= done						Indicates completion.
    """

    def __init__(self, topic='/hololens/navgoal'):
        """
        Constructor
        """
        super(WaitForRosNavgoalState, self).__init__(outcomes=['done'], output_keys=['navgoal'])
        self._sub = ProxySubscriberCached()
        self._topic = topic

    def execute(self, userdata):
        """Execute this state"""
        if self._navgoal is not None:
            userdata.navgoal = self._navgoal
            return 'done'

    def _navgoal_callback(self, msg):
        Logger.loginfo('navgoal callback on topic %s:%s' % (self._topic, str(msg.pose.position).replace('\n', ' ')))
        self._navgoal = msg

    def on_enter(self, userdata):
        self._navgoal = None
        self._sub.subscribe(self._topic, PoseStamped, self._navgoal_callback)

    def on_stop(self):
        self._sub.unsubscribe_topic(self._topic)

    def on_exit(self, userdata):
        self._sub.unsubscribe_topic(self._topic)
