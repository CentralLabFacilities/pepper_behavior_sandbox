#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from move_base_msgs.msg import *

"""
Created on 11/19/2015

@author: Spyros Maniatopoulos
"""


class MoveBaseState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    ># waypoint     PoseStamped      Target waypoint for navigation.
    <# head_angle   int              Head-Pose for navigation.

    <= arrived                  Navigation to target pose succeeded.
    <= failed                   Navigation to target pose failed.
    """

    def __init__(self, head_angle=25):
        """Constructor"""
        super(MoveBaseState, self).__init__(outcomes=['arrived', 'failed'],
                                            input_keys=['waypoint'])
        self._head_angle = head_angle
        self._head_topic = '/pepper_robot/head/pose'
        ProxyPublisher.createPublisher(self._pub, self._head_topic, String)

        self._action_topic = "/move_base"
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})

        self._arrived = False
        self._failed = False

    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._arrived:
            return 'arrived'
        if self._failed:
            return 'failed'

        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._arrived = True
                Logger.loginfo('Arrived at (%.1f,%.1f) [%s]' % (
                    userdata.waypoint.pose.position.x, userdata.waypoint.pose.position.y,
                    userdata.waypoint.header.frame_id))
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.loginfo('Could not reach (%.1f,%.1f) [%s]' % (
                    userdata.waypoint.pose.position.x, userdata.waypoint.pose.position.y,
                    userdata.waypoint.header.frame_id))
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'

    def on_enter(self, userdata):
        """Create and send action goal"""

        self._arrived = False
        self._failed = False

        # Create and populate action goal
        goal = MoveBaseGoal()
        goal.target_pose = userdata.waypoint

        # Set Head Pose (Down = '25:0')
        self._pub.publish(self._head_topic, '%d:0' % self._head_angle)

        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, goal)
            Logger.loginfo('Started Navigation to (%.1f,%.1f) [%s]' % (
                    userdata.waypoint.pose.position.x, userdata.waypoint.pose.position.y,
                    userdata.waypoint.header.frame_id))
        except Exception as e:
            Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
            self._failed = True

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')

    def on_exit(self, userdata):
        self.cancel_active_goals()

        # Set Head Pose Normal
        self._pub.publish(self._head_topic, '0:0')

    def on_stop(self):
        self.cancel_active_goals()

        # Set Head Pose Normal
        self._pub.publish(self._head_topic, '0:0')
