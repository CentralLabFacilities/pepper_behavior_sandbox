import smach
import rospy
from actuators.head_control import HeadControlPepper


class MoveHeadPepper(smach.State):
    def __init__(self, controller, _hv=None, _hh=None, wait=5):
        self.hv = _hv
        self.hh = _hh
        if _hv or _hh:
            input_k = []
        else:
            input_k = ['head_vertical','head_horizontal']
        smach.State.__init__(self, outcomes=['success'], input_keys=input_k)
        self.wait = wait
        self.headcontrol = controller
        rospy.sleep(1)
    def execute(self, userdata):
        if self.hv or self.hh:
            result = self.headcontrol.set_head(self.hv, self.hh)
        else:
            result = self.headcontrol.set_head(userdata.head_vertical, userdata.head_horizontal)
        rospy.sleep(self.wait)
        return result