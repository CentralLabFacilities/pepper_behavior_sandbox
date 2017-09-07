import rospy
import smach


class MoveHeadPepper(smach.State):
    def __init__(self, controller, _hv=None, _hh=None, wait=5, speed=0.05, output=False):
        self.hv = _hv
        self.hh = _hh
        self.output = output
        self.speed = speed
        output = []
        if self.output == True:
            output = ['horizontal_angle']
        if _hv and _hh:
            input_k = []
        elif _hv:
            input_k = ['head_horizontal']
        elif _hh:
            input_k = ['head_vertical']
        else:
            input_k = ['head_vertical', 'head_horizontal']
        smach.State.__init__(self, outcomes=['success'], input_keys=input_k, output_keys=output)
        self.wait = wait
        self.headcontrol = controller

    def execute(self, userdata):
        horizontal_angle = 0.0
        if self.hv and self.hh:
            horizontal_angle = self.hh
            result = self.headcontrol.set_head(self.hv, self.hh, self.speed)
        elif self.hv:
            horizontal_angle = userdata.head_horizontal
            result = self.headcontrol.set_head(self.hv, userdata.head_horizontal, self.speed)
        elif self.hh:
            horizontal_angle = self.hh
            result = self.headcontrol.set_head(userdata.head_vertical, self.hh, self.speed)
        else:
            horizontal_angle = userdata.head_horizontal
            result = self.headcontrol.set_head(userdata.head_vertical, userdata.head_horizontal, self.speed)
        if self.output == True:
            userdata.horizontal_angle = horizontal_angle
        rospy.sleep(self.wait)
        return result
