import rospy
import smach


class AnimationPlayerPepper(smach.State):
    def __init__(self, animation, controller, wait=0):
        self.wait = wait
        self.animation = animation
        smach.State.__init__(self, outcomes=['success'])
        self.pub = controller

    def execute(self, userdata):
        self.pub.publish_animation(self.animation)
        rospy.sleep(self.wait)
        return 'success'
