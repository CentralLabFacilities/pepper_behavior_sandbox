#!/usr/bin/env python

import time
import rospy
import smach
import smach_ros
import actionlib
from std_msgs.msg import String, Bool
from naoqi_bridge_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackActionGoal, SpeechWithFeedbackGoal


class DataActuators:
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
        self.speech_sub = rospy.Subscriber("/pepper_robot/speechrec/context", String, self.context_callback)
        self.current_context = ""

    def reset_context(self):
        self.current_context = ""

    def context_callback(self, data):
        self.current_context = data.data.strip()
        rospy.loginfo(self.current_context)


class RejectSpeech(smach.State):
    def __init__(self, _ds, _da):
        smach.State.__init__(self, outcomes=['finished', 'none'], input_keys=['text'])
        self.ds = _ds
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State RejectSpeech')
        while self.ds.current_context == "":
            time.sleep(0.1)
        if self.ds.current_context == "Pepper hier her":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            self.da.say_something("Sorry, ich muss Leute begruessen.")
            userdata.text = "Pepper hier her"
            return 'finished'
        elif self.ds.current_context == "Pepper kannst du tanzen":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            self.da.say_something("Die Uni ist doch keine Tanzschule!")
            userdata.text = "finished"
            return 'finished'
        elif self.ds.current_context == "Pepper guck doch mal":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            self.da.say_something("Ich kann leider noch keine zwei Dinge zur selben Zeit tun")
            userdata.text = "Pepper guck doch mal"
            return 'finished'
        elif self.ds.current_context == "Pepper was kannst du noch":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            self.da.say_something("Ich habe gerade eine Aufgabe, ich kann noch nicht zwei Dinge gleichzeitig")
            userdata.text = "Pepper was kannst du noch"
            return 'finished'
        elif self.ds.current_context == "Pepper schau doch mal":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            self.da.say_something("Ich habe gerade eine Aufgabe, ich kann noch nicht zwei Dinge gleichzeitig")
            userdata.text = "Pepper schau doch mal"
            return 'finished'
        rospy.logwarn('No valid command')
        self.ds.reset_context()
        return 'none'


def main():
    rospy.init_node('pepper_reject_speech')
    ds = DataSensors()
    da = DataActuators()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.text = ""

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('REJECT', RejectSpeech(ds, da),
                               transitions={'finished': 'REJECT', 'none': 'REJECT'})

    # Introspection viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/REJECT_SPEECH')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
