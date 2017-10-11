#!/usr/bin/env python

import qi
import sys
from optparse import OptionParser
import rospy
from std_msgs.msg import String
import time
import actionlib
from naoqi_bridge_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackGoal


class TalkControllerPepper:
    def __init__(self, sim=False):
        self.sim = sim
        if sim:
            self.head_pub = rospy.Publisher('/talk', String, queue_size=1)
        else:
            self.speech_as = actionlib.SimpleActionClient('/naoqi_tts_feedback', SpeechWithFeedbackAction)
            self.speech_as.wait_for_server(rospy.Duration(2))
        rospy.loginfo("Connected to Speech Client.")

    def say_something(self, _text):
        tts_goal = SpeechWithFeedbackGoal()
        tts_goal.say = _text
        self.speech_as.send_goal(tts_goal)



class Video_introduction(object):
    def __init__(self, appl):
        super(Video_introduction, self).__init__()
        appl.start()
        session = appl.session
        self.memory = session.service("ALMemory")
        self.motion = session.service("ALMotion")
        self.animation = session.service("ALAnimationPlayer")
        self.pubAnimation = rospy.Publisher("/pepper_robot/head/pose", String, queue_size=1)
        self.pubHead = rospy.Publisher("/pepper_robot/animation_player", String, queue_size=1)
        self.tts = TalkControllerPepper()

    # (re-) connect to NaoQI:

    def run(self):
        self.motion.moveTo(0,-0.4,0)
        self.tts.say_something("Hello! This is my second brain! This is some more filler text")
        self.pubHead("0:-70:0")
        time.sleep(1)
        self.pubAnimation("animation/Stand/Gestures/ShowSky_5")
        time.sleep(3)
        self.pubHead("0:0:0")
        self.tts.say_something("This is the end.")



if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--pip", dest="pip", default=True)
    parser.add_option("--pport", dest="pport", default=True)
    (options, args) = parser.parse_args()
    try:
        connection_url = options.pip + ":" + options.pport
        app = qi.Application(["PepperVideoIntroduction", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi")
        sys.exit(1)

    vi = Video_introduction(app)
    vi.run()
