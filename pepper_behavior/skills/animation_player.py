import rospy
import smach


class AnimationPlayerPepper(smach.State):
    def __init__(self, controller, id=None, wait=0, animationblock='greetings'):
        self.wait = wait
        self.animationblock = animationblock
        if animationblock == 'greetings':
            self.animation = ['animations/Stand/Gestures/Hey_1',
                            'animations/Stand/Gestures/Hey_3',
                            'animations/Stand/Gestures/Hey_4',
                            'animations/Stand/Gestures/Hey_6']
        else:
            self.animation = ['animations/Stand/Gestures/Thinking_1',
                              'animations/Stand/Gestures/Thinking_3',
                              'animations/Stand/Gestures/Thinking_4',
                              'animations/Stand/Gestures/Thinking_6']
        self.id = id
        if self.id:
            input_k = []
        else:
            input_k = ['id']
        smach.State.__init__(self, outcomes=['success'], input_keys=input_k)
        self.pub = controller

    def execute(self, userdata):
        if self.id:
            self.pub.publish_animation(self.animation[self.id])
        else:
            print("Animation: %s " % self.animation[self.id])
            self.pub.publish_animation(self.animation[userdata.id])
        rospy.sleep(self.wait)
        return 'success'
