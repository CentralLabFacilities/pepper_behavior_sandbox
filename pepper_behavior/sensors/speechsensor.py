import rospy
from std_msgs.msg import String
from clf_perception_vision.msg import ExtendedPeople


class SpeechSensor():
    def __init__(self):
        self.callback = rospy.Subscriber("/pepper_robot/speechrec/context", String,
                                              self.callback)
        print("Speech Sensor")

    def callback(self, data):
        self.text = []
        self.text = data.data

    def clean(self):
        self.text = []

    def getCmd(self):
        self.returnval = self.text
        self.clean(self)
        return self.returnval