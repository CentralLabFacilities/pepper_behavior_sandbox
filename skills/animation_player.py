import smach
import rospy
from actuators.ros_string_pub import RosStringPub


class AnimationPlayerPepper(smach.State):
    def __init__(self, scope, animation, wait=5):
        self.scope = scope
        self.wait = wait
        self.animation = animation
        smach.State.__init__(self, outcomes=['success'])
        self.pub = RosStringPub(self.scope)
        rospy.sleep(1)
    def execute(self, userdata):
        self.pub.publish_animation(self.animation)
        rospy.sleep(self.wait)
        return 'success'
