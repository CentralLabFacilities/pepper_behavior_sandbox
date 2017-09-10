#!/usr/bin/env python

import time
import rospy
import smach
import smach_ros
import actionlib
from std_msgs.msg import String, Bool
from math import radians, degrees
from people_msgs.msg import Person, People
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from naoqi_bridge_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackActionGoal, SpeechWithFeedbackGoal


class DataAcutators:
    def __init__(self):
        self.speech_as = actionlib.SimpleActionClient('/naoqi_tts_feedback', SpeechWithFeedbackAction)
        self.speech_as.wait_for_server()
        rospy.loginfo("Connected to SpeechServer")

    def say_something(self, _text):
        tts_goal = SpeechWithFeedbackGoal()
        tts_goal.say = _text
        self.speech_as.send_goal(tts_goal)
        rospy.loginfo("Waiting for result...")
        self.speech_as.wait_for_result()
        result = str(self.speech_as.get_state())
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


class RejectSpeech(smach.State):
    def __init__(self, _ds, _da):
        smach.State.__init__(self, outcomes=['finished', 'none'], input_keys=['text'])
        self.ds = _ds
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State RejectSpeech')
        self.da.set_head_down()
        while self.ds.current_context == "":
            time.sleep(0.1)
        if self.ds.current_context == "Pepper hier her":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.text = "Pepper hier her"
            return 'finished'
        elif self.ds.current_context == "Pepper tanz doch mal":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.text = "finished"
            return 'finished'
        elif self.ds.current_context == "Pepper guck doch mal":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.text = "Pepper guck doch mal"
            return 'finished'
        elif self.ds.current_context == "Pepper was kannst du noch":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.text = "Pepper was kannst du noch"
            return 'finished'
        rospy.logwarn('No valid command')
        self.ds.reset_context()
        return 'none'


def main():
    rospy.init_node('geniale_pepper_state_machine')
    ds = DataSensors()
    da = DataAcutators()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.go_to_goal = ""

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('REJECT', RejectSpeech(ds, da),
                               transitions={'finished': 'REJECT', 'none': 'REJECT', })

    # Introspection viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/REJECT_SPEECH')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
