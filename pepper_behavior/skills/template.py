import smach

class Template(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], input_keys=['input'], output_keys=['output'])
    def execute(self, userdata):
        return 'success'
