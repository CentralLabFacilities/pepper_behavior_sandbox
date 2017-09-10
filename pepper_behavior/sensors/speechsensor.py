import rospy
from std_msgs.msg import String


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
        return self.returnval