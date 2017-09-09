import smach
import rospy
import tf
import math


class CalculatePersonPosition(smach.State):
    def __init__(self, controller, max_distance=2.5, onlyhorizontal = False):
        self.person_sensor = controller
        self.max_distance = max_distance
        self.counter = 0
        self.tf = tf.TransformListener()
        input = []
        self.onlyhorizontal = onlyhorizontal
        if onlyhorizontal:
            input = ['old_vertical']
        smach.State.__init__(self, input_keys=input, outcomes=['success', 'repeat', 'no_person_found'],
                             output_keys=['person_angle_vertical', 'person_angle_horizontal'])

    def execute(self, userdata):
        self.person = None
        self.person = self.person_sensor.getDetectedPerson()
        self.pose = None
        self.dist = self.max_distance
        for p in self.person:
            pose = p.pose
            dist = distance(pose.pose.position)
            if dist < self.dist:
                try:
                    self.tf.waitForTransform('base_link', 'CameraDepth_optical_frame', rospy.Time.now(), rospy.Duration(4.0))
                    p = self.tf.transformPose("base_link", pose)
                    self.dist = dist
                    self.pose = p.pose
                except Exception:
                    print("Exception")
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
            return 'success'
        elif self.counter > 5:
            self.counter = 0
            return 'no_person_found'
        else:
            rospy.sleep(1)
            self.counter = self.counter + 1
            return 'repeat'


def distance(trans):
    dist = math.sqrt(trans.x * trans.x + trans.y * trans.y + trans.z * trans.z)
    return dist


def rotation(pose):
    print ("orientation")
    x = pose.position.x+ 0.0133
    y = pose.position.y + 0.039
    z = pose.position.z - 0.288 + 0.2
    print(x)
    print(y)
    print(z)
    horizontal = math.degrees(math.atan2(y, x))
    vertical = math.degrees(math.atan2(z, x))
    print(horizontal)
    print(vertical)
    return (vertical, horizontal)
