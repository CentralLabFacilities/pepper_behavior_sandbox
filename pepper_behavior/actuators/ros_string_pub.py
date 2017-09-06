import rospy
from std_msgs.msg import String


class RosStringPub:
    def __init__(self, scope):
        self.scope = scope
        self.pub = rospy.Publisher(scope, String, queue_size=1)
        rospy.loginfo("Connected to %s." % scope)

    def publish_animation(self, animation):
        result = self.pub.publish(animation)
        return result
