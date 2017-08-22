#!/usr/bin/env python

import time
import rospy
import smach
import roslib
import smach_ros
import actionlib
from std_msgs.msg import String
from math import radians, degrees
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from naoqi_bridge_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackActionGoal, SpeechWithFeedbackGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class DataAcutators:
    def __init__(self):
        self.nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.speech_as = actionlib.SimpleActionClient('/naoqi_tts_feedback', SpeechWithFeedbackAction)
        rospy.loginfo("Connecting to /move_base...")
        self.nav_as.wait_for_server()
        rospy.loginfo("Connected.")

    def set_nav_goal(self, x, y, q0, q1, q2, q3):
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = '/map'  # Note: the frame_id must be map
        mb_goal.target_pose.pose.position.x = x
        mb_goal.target_pose.pose.position.y = y
        mb_goal.target_pose.pose.position.z = 0.0  # z must be 0.0 (no height in the map)
        # Orientation of the robot is expressed in the yaw value of euler angles
        mb_goal.target_pose.pose.orientation = Quaternion(q0, q1, q2, q3)
        self.nav_as.send_goal(mb_goal)
        rospy.loginfo("Waiting for result...")
        self.nav_as.wait_for_result()
        result = str(self.nav_as.get_state())
        # 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
        return result

    def say_something(self, _text):
        tts_goal = SpeechWithFeedbackGoal()
        tts_goal.say = _text
        self.speech_as.send_goal(tts_goal)
        rospy.loginfo("Waiting for result...")
        self.speech_as.wait_for_result()
        result = str(self.nav_as.get_state())
        # 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
        return result


class DataSensors:
    def __init__(self):
        self.speech_rec_context = rospy.Subscriber("/pepper_robot/speechrec/context", String, self.context_callback)
        self.current_context = ""

    def reset_context(self):
        self.current_context = ""

    def context_callback(self, data):
        self.current_context = data.data.strip()


class WaitForCommand(smach.State):
    def __init__(self, _ds):
        smach.State.__init__(self, outcomes=['table', 'init', 'age', 'what', 'gender', 'none'])
        self.ds = _ds

    def execute(self, userdata):
        rospy.loginfo('Entering State WaitForCommand')
        while self.ds.current_context == "":
            time.sleep(0.1)
        if self.ds.current_context == "Drive to the table":
            rospy.loginfo('Drive to the table')
            self.ds.reset_context()
            return 'table'
        elif self.ds.current_context == "Go back to init position":
            rospy.loginfo('Go back to init position')
            self.ds.reset_context()
            return 'init'
        rospy.logwarn('No valid command')
        self.ds.reset_context()
        return 'none'


class GoToTable(smach.State):
    def __init__(self, _da):
        smach.State.__init__(self, outcomes=['arrived', 'fail'])
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State GoToTable')
        self.da.say_something("Okay my friend, I will drive to the table")
        goal = self.da.set_nav_goal(5.98, -6.39, 0.0, 0.0, 0.99, -0.12)
        if goal == "3":
            self.da.say_something("I am done. What shall I do?")
            return 'arrived'
        else:
            self.da.say_something("I could not reach the table, oooooh no.")
            rospy.logwarn('Could not reach table')
            return 'fail'


class GoToInit(smach.State):
    def __init__(self, _da):
        smach.State.__init__(self, outcomes=['arrived', 'fail'])
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State GoToInit')
        self.da.say_something("Well, going back.")
        goal = self.da.set_nav_goal(8.77, -4.94, 0.0, 0.0, 0.94, -0.31)
        if goal == "3":
            self.da.say_something("I am at my init position. What can I do for you?")
            return 'arrived'
        else:
            self.da.say_something("I could not reach my init position")
            rospy.logwarn('Could not reach init')
            return 'fail'


def main():
    rospy.init_node('geniale_pepper_state_machine')
    ds = DataSensors()
    da = DataAcutators()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT', WaitForCommand(ds),
                               transitions={'table': 'GOTOTABLE',
                                            'init': 'GOTOINIT',
                                            'age': 'GOTOTABLE',
                                            'what': 'GOTOTABLE',
                                            'gender': 'GOTOTABLE',
                                            'none': 'WAIT'})
        smach.StateMachine.add('GOTOTABLE', GoToTable(da),
                               transitions={'arrived': 'WAIT',
                                            'fail': 'WAIT'})

        smach.StateMachine.add('GOTOINIT', GoToInit(da),
                               transitions={'arrived': 'WAIT',
                                            'fail': 'WAIT'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
