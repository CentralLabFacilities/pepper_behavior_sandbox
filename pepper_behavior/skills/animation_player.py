import rospy
import smach


class AnimationPlayerPepper(smach.State):
    def __init__(self, id, controller, wait=0):
        self.wait = wait
        self.animation = ['animations/Stand/Gestures/Hey_1',
                          'animations/Stand/Gestures/Hey_3',
                          'animations/Stand/Gestures/Hey_4',
                          'animations/Stand/Gestures/Hey_6']
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
            print(userdata.id)
            self.pub.publish_animation(self.animation[userdata.id])
        rospy.sleep(self.wait)
        return 'success'
