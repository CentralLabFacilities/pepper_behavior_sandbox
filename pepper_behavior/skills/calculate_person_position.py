import smach
import rospy
import tf
import math
import random


class CalculatePersonPosition(smach.State):
    def __init__(self, controller, controller_2=None, sensor=None, max_distance=2.5, onlyhorizontal=False, knownperson=True):
        self.person_sensor = controller
        self.max_distance = max_distance
        self.person_id = sensor
        self.talk_known = controller_2
        self.ignoreknownperson = knownperson
        self.talks =  ['Oh, ich denke Dich habe ich schon begruesst',
                       'Dich kenne ich schon, ich mache weiter',
                       'Oh schoen dich wieder zu sehen, bis gleich.',
                       'Wen haben wir denn da, dich kenne ich. Gleich gehen die Vortraege los!',
                       'Hallo noch mal. Wie waere es wenn uns uns nachher unterhalten?']
        self.counter = 0
        # https://answers.ros.org/question/10777/service-exception-using-tf-listener-in-rospy
        self.tf = tf.TransformListener()
        input = []
        self.onlyhorizontal = onlyhorizontal
        if onlyhorizontal:
            input = ['old_vertical']
        smach.State.__init__(self, input_keys=input, outcomes=['success', 'repeat', 'no_person_found', 'known'],
                             output_keys=['person_angle_vertical', 'person_angle_horizontal'])

    def execute(self, userdata):
        self.person = None
        self.person_sensor.clearPerson()
        rospy.sleep(0.1)
        self.person = self.person_sensor.getDetectedPerson()
        self.pose = None
        self.transformid = None
        self.dist = self.max_distance
        for p in self.person:
            pose = p.pose
            dist = distance(pose.pose.position)
            if dist < self.dist:
                try:
                    self.tf.waitForTransform('base_link', 'CameraDepth_optical_frame', rospy.Time.now(),
                                             rospy.Duration(4.0))
                    po = self.tf.transformPose("base_link", pose)
                    self.dist = dist
                    self.pose = po.pose
                    self.transformid = p.transformid
                except Exception, e:
                    print("Exception %s" % e)
                    return 'repeat'
            if dist > self.max_distance and self.max_distance == self.dist:
                print('Detected person to far away. Distance: %s ' % dist)
        if self.pose:
            (vertical, horizontal) = rotation(self.pose)
            self.counter = 0
            if self.onlyhorizontal:
                userdata.person_angle_vertical = userdata.old_vertical
            else:
                userdata.person_angle_vertical = vertical
            userdata.person_angle_horizontal = horizontal
            if self.dist < 1.8 and self.transformid is not None:
                try:
                    known, name = self.person_id.identify(self.transformid)
                    if known and self.ignoreknownperson:
                        rospy.loginfo("Person is known, iterating")
                        if self.talk_known is not None:
                            self.talk_known.say_something(random.choice(self.talks))
                        return 'known'
                    else:
                        return 'success'
                except Exception, e:
                    rospy.logwarn("Something went wront while identifying a person")
            return 'success'
        elif self.counter > 5:
            self.counter = 0
            return 'no_person_found'
        else:
            self.counter = self.counter + 1
            return 'repeat'


def distance(trans):
    dist = math.sqrt(trans.x * trans.x + trans.y * trans.y + trans.z * trans.z)
    return dist


def rotation(pose):
    print ("orientation")
    x = pose.position.x + 0.0133
    y = pose.position.y
    z = pose.position.z - 0.288 + 0.2
    print(x)
    print(y)
    print(z)
    horizontal = math.degrees(math.atan2(y, x))
    vertical = math.degrees(math.atan2(-z, x))
    print(horizontal)
    print(vertical)
    return (vertical, horizontal)
