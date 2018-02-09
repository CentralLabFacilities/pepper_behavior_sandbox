import rospy
from clf_perception_vision_msgs.srv import *


class IdSensor:
    def __init__(self):
        rospy.wait_for_service('pepper_face_identification')
        rospy.loginfo("Init Person ID Sensor")

    def identify(self, _data):
        try:
            person_id = rospy.ServiceProxy('pepper_face_identification', DoIKnowThatPerson)
            resp = person_id(_data)
            return resp.known, resp.name
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)
