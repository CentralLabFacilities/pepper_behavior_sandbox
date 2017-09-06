import rospy
import math
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class LeftArmControlPepper:
    def __init__(self):
        self.position = JointAnglesWithSpeed()
        self.position.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
        self.position.joint_angles = [0.0, 20.0, 0.0, 5.0, 0.0]
        self.head_pub = rospy.Publisher('/pepper_robot/set/arm/left/tilt', JointAnglesWithSpeed, queue_size=1)
        rospy.loginfo("Connected to Arm Control.")

    def set_arm(self, gesture):
        if gesture == 'test':
            self.position.joint_angles = [math.radians(0.0), math.radians(45.0), math.radians(0.0), math.radians(5.0),
                                          math.radians(0.0)]
        else:
            return 'unknown_gesture'
        self.position.speed = 0.1
        self.head_pub.publish(self.position)
        return 'success'
