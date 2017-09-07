import rospy
import smach


class MoveHeadPepper(smach.State):
    def __init__(self, controller, _hv=None, _hh=None, wait=5, speed=0.05):
        self.hv = _hv
        self.hh = _hh
        self.speed = speed
        if _hv and _hh:
            input_k = []
        elif _hv:
            input_k = ['head_horizontal']
        elif _hh:
            input_k = ['head_vertical']
        else:
            input_k = ['head_vertical', 'head_horizontal']
        smach.State.__init__(self, outcomes=['success'], input_keys=input_k)
        self.wait = wait
        self.headcontrol = controller

    def execute(self, userdata):
        if self.hv and self.hh:
            result = self.headcontrol.set_head(self.hv, self.hh, self.speed)
        elif self.hv:
            result = self.headcontrol.set_head(self.hv, userdata.head_horizontal, self.speed)
        elif self.hh:
            result = self.headcontrol.set_head(userdata.head_vertical, self.hh, self.speed)
        else:
            result = self.headcontrol.set_head(userdata.head_vertical, userdata.head_horizontal, self.speed)
        rospy.sleep(self.wait)
        return result
