import smach


class Iterate(smach.State):
    def __init__(self, iterationsteps):
        # Number off different outputs. If diff_outputs = 3 it returns success_0, success_1 and success_2
        self.diff_outputs = iterationsteps
        out = []
        for i in range(0, self.diff_outputs):
            out.append('success_' + str(i))
        smach.State.__init__(self, outcomes=out, input_keys=['iterate_input'], output_keys=['iterate_output'])

    def execute(self, userdata):
        print(userdata.iterate_input)
        nextmode = userdata.iterate_input + 1
        if nextmode == self.diff_outputs:
            nextmode = 0
            userdata.iterate_output = nextmode
            return 'success_' + str(0)
        else:
            userdata.iterate_output = nextmode
            return 'success_' + str(nextmode)


class Counter(smach.State):
    def __init__(self, numbers, reset=False):
        self.reset = reset
        self.diff_outputs = numbers
        out = ['success', 'end']
        smach.State.__init__(self, outcomes=out, input_keys=['counter_input'], output_keys=['counter_output'])

    def execute(self, userdata):
        if self.reset == True:
            userdata.counter_output = 0
            return 'success'
        nextmode = userdata.counter_input + 1
        if nextmode > self.diff_outputs:
            nextmode = 0
            userdata.counter_output = nextmode
            return 'end'
        else:
            userdata.counter_output = nextmode
            return 'success'