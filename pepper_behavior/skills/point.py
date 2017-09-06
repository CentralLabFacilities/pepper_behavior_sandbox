import smach
import rospy


class LeftArmGesture(smach.State):
    def __init__(self, controller, gesture, wait=5):
        self.controller = controller
        self.gesture = gesture
        self.wait = wait
        smach.State.__init__(self, outcomes=['success', 'unknown_gesture'])

    def execute(self, userdata):
        result = self.set_arm.set_head(self.gesture)
        rospy.sleep(self.wait)
        return result
