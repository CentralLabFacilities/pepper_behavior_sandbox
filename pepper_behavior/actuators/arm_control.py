import rospy
import math
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class LeftArmControlPepper:
    def __init__(self):
        self.position = JointAnglesWithSpeed()
        self.arm_pub = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=1)
        rospy.loginfo("Connected to Arm Control.")

    def set_arm(self, gesture):
        if gesture == 'test':
            self.position.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
            self.position.joint_angles = [math.radians(0.0), math.radians(45.0), math.radians(0.0), math.radians(5.0),
                                          math.radians(0.0)]
        elif gesture == 'demo':
            self.position.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            self.position.joint_angles = [math.radians(0.0), math.radians(-45.0), math.radians(0.0), math.radians(-5.0),
                                      math.radians(0.0)]
        elif gesture == 'point_demo':
            self.position.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            self.position.joint_angles = [math.radians(95.8), math.radians(-6.6), math.radians(83.8), math.radians(-16.2),
                math.radians(1.4)]
        else:
            return 'unknown_gesture'
        self.position.speed = 0.1
        print(self.position)
        self.arm_pub.publish(self.position)
        return 'success'