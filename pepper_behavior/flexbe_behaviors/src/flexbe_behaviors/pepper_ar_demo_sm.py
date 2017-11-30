#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from pepper_flexbe_states.wait_for_ros_navgoal import WaitForRosNavgoalState
from pepper_flexbe_states.set_navgoal_state import MoveBaseState
from pepper_flexbe_states.talk_state import TalkState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Do Nov 30 2017
@author: Felix Friese
'''
class PepperARDemoSM(Behavior):
	'''
	Hololens AR-Demo for Pepper
	'''


	def __init__(self):
		super(PepperARDemoSM, self).__init__()
		self.name = 'Pepper AR Demo'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:263 y:359, x:575 y:244
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:84 y:39
			OperatableStateMachine.add('WaitForHololensGoal',
										WaitForRosNavgoalState(topic='/hololens/navgoal'),
										transitions={'done': 'DriveToHololensGoal'},
										autonomy={'done': Autonomy.Off},
										remapping={'navgoal': 'navgoal'})

			# x:299 y:138
			OperatableStateMachine.add('DriveToHololensGoal',
										MoveBaseState(),
										transitions={'arrived': 'finished', 'failed': 'SayWayBlocked'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'navgoal'})

			# x:576 y:14
			OperatableStateMachine.add('SayWayBlocked',
										TalkState(message='My way is blocked. Please clear the way and call me again', blocking=True),
										transitions={'done': 'WaitForHololensGoal', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
