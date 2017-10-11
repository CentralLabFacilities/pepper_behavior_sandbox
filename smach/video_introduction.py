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

    def say_something_blocking(self, _text):
        tts_goal = SpeechWithFeedbackGoal()
        tts_goal.say = _text

        if self.sim:
            self.head_pub.publish(_text)
            rospy.sleep(2)
            result = 'success'
        else:
            self.speech_as.send_goal(tts_goal)
            result = self.speech_as.wait_for_result()
            result = 'success'  # remove and use real results
        return result



class Video_introduction(object):
    def __init__(self, appl):
        super(Video_introduction, self).__init__()
        appl.start()
        session = appl.session
        self.memory = session.service("ALMemory")
        self.motion = session.service("ALMotion")
        self.pubAnimation = rospy.Publisher("/pepper_robot/animation_player", String, queue_size=1)
        self.pubHead = rospy.Publisher("/pepper_robot/head/pose", String, queue_size=1)
        self.tts = TalkControllerPepper()

    # (re-) connect to NaoQI:

    def run(self):
        self.motion.moveTo(0,-0.4,0)
        self.tts.say_something_blocking("Hello! My name is Tobi! I would like to participate in the RoboCup 2018! I will shortly introduce my system architecture!")
        self.tts.say_something("ROS is running on my head, wrapping NaoQi!")
        self.pubAnimation.publish("animations/Stand/Gestures/But_1")
        time.sleep(2)
        self.tts.say_something_blocking("For example: The ROS navigation stack is deployed on my head, this enables me to navigate autonomously!")
        self.tts.say_something_blocking("I am also grabbing and streaming my camera inputs compressed using ROS")
        self.pubHead.publish("0:-70:0")
        time.sleep(1)
        self.tts.say_something("This is my Laptop and the only external computing resource! Additional components like behavior coordination, object recognition and person perception are running on it!")
        self.pubAnimation.publish("animations/Stand/Gestures/ShowSky_5")
        time.sleep(3)
        self.pubHead.publish("0:0:0")
        time.sleep(5)
        self.pubAnimation.publish("animations/Stand/Gestures/Me_7")
        self.tts.say_something_blocking("Lastly, I want to show you my tablet! Where additional information is displayed and which can be used to interact with me.")
        self.tts.say_something_blocking("Now I am going to show you some of my skills with the help of Felix and Kai!")
        self.tts.say_something_blocking("Please remember, I am not the fastest driving robot. But people are actively working on that!")



if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--pip", dest="pip", default=True)
    parser.add_option("--pport", dest="pport", default=True)
    rospy.init_node('pepper_video_introduction', anonymous=True)
    (options, args) = parser.parse_args()
    try:
        connection_url = options.pip + ":" + options.pport
        app = qi.Application(["PepperVideoIntroduction", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi")
        sys.exit(1)

    vi = Video_introduction(app)
    vi.run()
