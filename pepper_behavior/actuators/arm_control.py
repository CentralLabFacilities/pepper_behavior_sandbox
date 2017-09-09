import rospy
import math
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class LeftArmControlPepper:
    def __init__(self):
        self.position = JointAnglesWithSpeed()
        self.arm_pub = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=1)
        rospy.loginfo("Connected to Arm Control.")

    def set_arm(self, gesture):
        if gesture == 'demo':
            self.position.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            self.position.joint_angles = [math.radians(3.6), math.radians(-57.0), math.radians(66.0), math.radians(20.9),
                                      math.radians(31.1)]
        else:
            return 'unknown_gesture'
        self.position.speed = 0.35
        self.arm_pub.publish(self.position)
        return 'success'
