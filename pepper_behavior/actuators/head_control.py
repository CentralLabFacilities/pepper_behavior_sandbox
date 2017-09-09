import rospy
import math
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class HeadControlPepper:
    def __init__(self):
        self.position = JointAnglesWithSpeed()
        self.position.joint_names = ['HeadPitch', 'HeadYaw']
        self.position.joint_angles = [0.0, 0.0]
        self.head_pub = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=1)
        rospy.loginfo("Connected to Head Control.")

    def set_head(self, vertical, horizontal, speed=0.05):
        # vertical: back up down drive
        # horizontal: strong_left left half_left center half_right right strong_right
        vertical_option = {'back': -10.0, 'up': 0.0, 'drive': 13.0, 'down': 25.0}
        vertical_parameter = vertical_option.get(vertical, vertical)
        horizontal_option = {'left': 45.0, 'center': 0.0, 'right': -45.0, 'half_left': 30.0, 'half_right': -30.0,
                             'strong_left': 60.0, 'strong_right': -60.0}
        horizontal_parameter = horizontal_option.get(horizontal, horizontal)
	if vertical_parameter < -10: 
		print("Vertical < -10, %s", vertical_parameter)
		vertical_parameter = - 10
		vertical = 10
	if vertical_parameter > 25:
		print("Vertical > 25, %s", vertical_parameter)
		vertical_parameter = 25
		vertical = 25
	if horizontal_parameter > 70:
		print("horizontal > 70, %s", horizontal_parameter)
		horizontal_parameter = 70
		horizontal = 70
	if horizontal_parameter < -70:
		print("horizontal < 70, %s", horizontal_parameter)
		horizontal_parameter = -70
		horizontal = -70
        self.position.joint_angles = [math.radians(vertical_parameter), math.radians(horizontal_parameter)]
        self.position.speed = speed
        rospy.loginfo("Set Head vertical: %s." % vertical)
        rospy.loginfo("Set Head horizontal: %s." % horizontal)
        self.head_pub.publish(self.position)
        return 'success'
