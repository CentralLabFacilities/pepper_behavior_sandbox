#!/usr/bin/env python
import rospy
import actionlib

from flexbe_core import EventState, Logger
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import String, Bool


class PepperExample(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self, target_time):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(PepperExample, self).__init__(outcomes = ['continue', 'failed'])

		
        	self.current_goal = MoveBaseGoal()
	        self.nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	        #self.head_pub = rospy.Publisher('/pepper_robot/head/pose', String, queue_size=1)

		rospy.loginfo("Connecting to /move_base...")
		self.nav_as.wait_for_server()
		rospy.loginfo("Connected.")


		# Store state parameter for later use.
		self._target_time = rospy.Duration(target_time)

		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		self._start_time = None


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		self.set_nav_goal(0.0, 0.0, 0.0 , 0.0 , 0.0 , 0.0 )
		#if rospy.Time.now() - self._start_time > self._target_time:
		#	return 'continue' # One of the outcomes declared above.

		Logger.loginfo('execute')
		return 'continue' # One of the outcomes declared above.
		

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.

		time_to_wait = (self._target_time - (rospy.Time.now() - self._start_time)).to_sec()

		if time_to_wait > 0:
			Logger.loginfo('Have to wait for %.1f seconds.' % time_to_wait)


	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		# In this example, we use this event to set the correct start time.
		self._start_time = rospy.Time.now()


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		

	def set_nav_goal(self, x, y, q0, q1, q2, q3):
		Logger.loginfo('enter navgoal')
		try:
		    #self.set_head_drive()
		    mb_goal = MoveBaseGoal()
		    mb_goal.target_pose.header = mb_goal.header
		    mb_goal.target_pose.header.frame_id = 'map'  # Note: the frame_id must be map
		    mb_goal.target_pose.pose.position.x = x
		    mb_goal.target_pose.pose.position.y = y
		    mb_goal.target_pose.pose.position.z = 0.0  # z must be 0.0 (no height in the map)
		    # Orientation of the robot is expressed in the yaw value of euler angles
		    mb_goal.target_pose.pose.orientation = Quaternion(q0, q1, q2, q3)
		    self.current_goal = mb_goal
		    self.nav_as.send_goal(mb_goal)
		    rospy.loginfo("Waiting for result...")
		    Logger.loginfo('waiting for result---')
		    self.nav_as.wait_for_result()
		    result = str(self.nav_as.get_state())
		    # 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
		except Exception, e:
		    Logger.loginfo(str(e))
		    return str(5)
		return result
