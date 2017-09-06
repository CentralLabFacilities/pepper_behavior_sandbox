import rospy
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


class BaseControlPepper:
    def __init__(self):
        self.current_goal = MoveBaseGoal()
        self.nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.nav_as.wait_for_server()
        rospy.loginfo("Connected to move_base.")

    def drive_to_nav_goal(self, x, y, q0, q1, q2, q3):
        try:
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = '/map'  # Note: the frame_id must be map
            mb_goal.target_pose.pose.position.x = x
            mb_goal.target_pose.pose.position.y = y
            mb_goal.target_pose.pose.position.z = 0.0  # z must be 0.0 (no height in the map)
            # Orientation of the robot is expressed in the yaw value of euler angles
            mb_goal.target_pose.pose.orientation = Quaternion(q0, q1, q2, q3)
            self.current_goal = mb_goal
            self.nav_as.send_goal(mb_goal)
            rospy.loginfo("Waiting for result...")
            self.nav_as.wait_for_result()
            result = str(self.nav_as.get_state())
            # 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
        except Exception, e:
            return str(5)
        return result

    def turn(self, direction, rotation):
        try:
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = '/base_link'  # Note: the frame_id must be map
            mb_goal.target_pose.pose.position.x = 0
            if direction == 'left':
                rospy.loginfo('Turn left for %s degrees.' % rotation)
                rotation = math.radians(rotation)
            elif direction == 'right':
                rospy.loginfo('Turn right for %s degrees.' % rotation)
                rotation = math.radians(-rotation)
            else:
                rospy.logerr('use direction left or right!')
                rotation = 0
            q = tf.transformations.quaternion_from_euler(0, 0, rotation)
            mb_goal.target_pose.pose.orientation = Quaternion(*q)
            self.current_goal = mb_goal
            self.nav_as.send_goal(mb_goal)
            rospy.loginfo("Waiting for result...")
            self.nav_as.wait_for_result()
            result = str(self.nav_as.get_state())
            # 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
            if result == 3:
                result = 'success'
            elif result == 4:
                result = 'aborted'
            elif result == 5:
                result = 'rejected'
            else:
                result = 'unknown'
        except Exception, e:
            return str(5)
        return result
