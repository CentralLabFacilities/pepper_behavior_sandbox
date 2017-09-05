import smach
import rospy
from actuators.base_control import BaseControlPepper


class TurnBasePepper(smach.State):
    def __init__(self, controller, orientation, degree):
        self.orientation = orientation
        self.degree = degree
        smach.State.__init__(self, outcomes=['success','aborted','rejected','unknown'])
        self.basecontrol = controller
        rospy.sleep(1)
    def execute(self, userdata):
        result = self.basecontrol.turn(self.orientation,self.degree)
        return result