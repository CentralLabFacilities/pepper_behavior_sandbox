import smach


class StatePublisher(smach.State):
    def __init__(self, controller, state, colorcontroller=None, color=None):
        self.controller = controller
        self.colorcont = colorcontroller
        self.color = color
        self.state = state
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userata):
        self.controller.publish(self.state)
        if self.colorcont and self.color:
            self.colorcont.publish(self.color)
        return 'success'
