import smach


class TurnBasePepper(smach.State):
    def __init__(self, controller, orientation, degree):
        self.orientation = orientation
        self.degree = degree
        smach.State.__init__(self, outcomes=['success', 'aborted', 'rejected', 'unknown'])
        self.basecontrol = controller

    def execute(self, userdata):
        result = self.basecontrol.turn(self.orientation, self.degree)
        return result
