import smach
import rospy
import actionlib
import tf
import math

class TurnWithoutMovebase(smach.State):
    def __init__(self, controller, angle):
	self.controller = controller
	self.angle = angle
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
	msg = '0:0:' + str(math.radians(self.angle))
	self.controller.publish(msg)
        return 'success'



