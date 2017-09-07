import smach
import rospy
import tf
import math


class CalculatePersonPosition(smach.State):
    def __init__(self, controller, max_distance=2.5):
        self.person_sensor = controller
        self.max_distance = max_distance
        self.counter = 0
        self.tf = tf.TransformListener()
        smach.State.__init__(self, outcomes=['success', 'repeat', 'no_person_found'],
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
                    rospy.sleep(2)
                    return 'repeat'
            if dist > self.max_distance and self.max_distance == self.dist:
                print('Detected person to far away. Distance: %s ' % dist)
        if self.pose:
            (vertical, horizontal) = rotation(self.pose)
            self.counter = 0
            userdata.person_angle_vertical = vertical
            userdata.person_angle_horizontal = horizontal
            return 'success'
        elif self.counter > 5:
            self.counter = 0
            return 'no_person_found'
        else:
            rospy.sleep(2)
            self.counter = self.counter + 1
            return 'repeat'


def distance(trans):
    dist = math.sqrt(trans.x * trans.x + trans.y * trans.y + trans.z * trans.z)
    return dist


def rotation(pose):
    print ("orientation")
    x = pose.position.x+ 0.0133
    y = pose.position.y + 0.039
    z = pose.position.z - 0.288 + 0.5
    print(x)
    print(y)
    print(z)
    horizontal = math.degrees(math.tan(y / x))
    vertical = math.degrees(math.tan(-z / x))
    print(horizontal)
    print(vertical)
    return (vertical, horizontal)
