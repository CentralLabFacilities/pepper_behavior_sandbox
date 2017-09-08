import smach


class StatePublisher(smach.State):
    def __init__(self, controller, state):
        self.controller = controller
        self.state = state
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userata):
        self.controller.publish(self.state)
        return 'success'
