
import smach
from pepper_behavior.sensors.person_sensor import PersonSensor
from clf_perception_vision.msg import ExtendedPersonStamped
from geometry_msgs.msg import Transform, PoseStamped, Quaternion
import rospy
import tf
from tf.transformations import euler_from_quaternion
import math


class CalculatePersonPosition(smach.State):
    def __init__(self,controller, max_distance=2.5):
        self.person_sensor = controller
        self.max_distance = max_distance
        self.counter = 0
        self.tf = tf.TransformListener()
        smach.State.__init__(self, outcomes=['success','repeat','no_person_found'],output_keys=['person_angle_vertical', 'person_angle_horizontal'])

    def execute(self, userdata):
        self.person = self.person_sensor.getDetectedPerson()
        self.pose = None
        self.dist = self.max_distance
        for p in self.person:
            pose = p.pose
            dist = distance(pose.pose.position)
            if dist < self.dist:
                self.tf.waitForTransform('/base_link', '/CameraDepth_optical_frame', rospy.Time(), rospy.Duration(4.0))
                p = self.tf.transformPose("base_link",pose)
                self.dist = dist
                self.pose = p.pose
        if self.pose:
            (vertical, horizontal) = rotation(self.pose)
            self.counter = 0
            userdata.person_angle_vertical = math.degrees(vertical)
            userdata.person_angle_horizontal = math.degrees(horizontal)
            return 'success'
        elif self.counter > 5:
            self.counter = 0
            return 'no_person_found'
        else:
            rospy.sleep(2)
            self.counter = self.counter + 1
            return 'repeat'


def distance(trans):
    dist = math.sqrt(trans.x*trans.x+trans.y*trans.y+trans.z*trans.z)
    return dist

def rotation(pose):
    print ("orientation")
    horizontal = math.tan(pose.position.y/pose.position.x)
    vertical = math.tan(pose.position.z/pose.position.x)
    print(horizontal)
    print(vertical)
    return (vertical, horizontal)