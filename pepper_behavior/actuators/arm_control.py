import rospy
import math
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class LeftArmControlPepper:
    def __init__(self):
        self.position = JointAnglesWithSpeed()
        self.head_pub = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=1)
        rospy.loginfo("Connected to Arm Control.")

    def set_left_arm(self, gesture):
        self.position.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
        if gesture == 'test':
            self.position.joint_angles = [math.radians(0.0), math.radians(45.0), math.radians(0.0), math.radians(5.0),
                                          math.radians(0.0)]
        else:
            return 'unknown_gesture'
        self.position.speed = 0.1
        print('left arm')
        print(self.position)
        self.head_pub.publish(self.position)
        return 'success'

    def set_right_arm(self, gesture):
        self.position.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        if gesture == 'point_demo':
            self.position.joint_angles = [math.radians(95.8), math.radians(-6.6), math.radians(83.8), math.radians(16.2),
                                          math.radians(1.4)]
        else:
            return 'unknown_gesture'
        self.position.speed = 0.1
        print('right arm')
        print(self.position)
        self.head_pub.publish(self.position)
        return 'success'