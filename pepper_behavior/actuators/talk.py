import rospy
import actionlib
from naoqi_bridge_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackGoal
from std_msgs.msg import String


class TalkControllerPepper:
    def __init__(self, sim=False):
        self.sim = sim
        if sim:
            self.head_pub = rospy.Publisher('/talk', String, queue_size=1)
        else:
            self.speech_as = actionlib.SimpleActionClient('/naoqi_tts_feedback', SpeechWithFeedbackAction)
            self.speech_as.wait_for_server()
        rospy.loginfo("Connected to Speech Client.")

    def say_something(self, _text):
        print("say")
        tts_goal = SpeechWithFeedbackGoal()
        tts_goal.say = _text

        if self.sim:
            self.head_pub.publish(_text)
            rospy.sleep(2)
            result = 'success'
        else:
            self.speech_as.send_goal(tts_goal)
            result = self.speech_as.wait_for_result()
        return result
