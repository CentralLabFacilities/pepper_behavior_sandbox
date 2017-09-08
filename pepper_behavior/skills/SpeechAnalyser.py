import smach
import rospy


class SpeechAnalyser(smach.State):
    def __init__(self, controller, wait=10):
        self.reset = 'Roboter setze dich zurueck'
        self.question = ['Pepper wie geht es dir',
                         'Pepper woher kommst du',
                         'Pepper was kannst du alles',
                         'Pepper was kann ich auf der itelligence World alles sehen'
                         'Wieso ist c eine schöne sprache'
                         'wieviele bits hat ein byte'
                         'wie lauten die drei gesetze der robotik']
        self.wait = wait
        self.controller = controller
        smach.State.__init__(self, outcomes=['success', 'no_cmd', 'reset'], output_keys=['msg_output'])

    def execute(self, userdata):
        self.controller.clean()
        userdata.msg_output = None
        for i in range(0, self.wait):
            cmd = self.controller.getCmd()
            rospy.sleep(1)
            if cmd:
                print('Got msg %s' %cmd)
                if cmd == self.reset:
                    return 'reset'
                for i in range(0,7):
                    print('Question %s' % self.question[i])
                    if cmd == self.question[i]:
                        userdata.msg_output = i
                        return 'success'
        print('No cmd.')
        return 'no_cmd'
