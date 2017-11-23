#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from pepper_flexbe_states.wait_for_naoqi_speech import WaitForNaoQiSpeechState
from pepper_flexbe_states.set_navgoal_state import MoveBaseState
from pepper_flexbe_states.generate_navgoal import GenerateNavgoalState
from pepper_flexbe_states.talk_state import TalkState
from pepper_flexbe_states.wait_for_open_door import WaitForOpenDoorState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Nov 17 2017
@author: Felix Friese
'''
class PepperDemoSM(Behavior):
	'''
	Simple Pepper Example
	'''


	def __init__(self):
		super(PepperDemoSM, self).__init__()
		self.name = 'Pepper Demo'

		# parameters of this behavior
		self.add_parameter('x', 1.0)
		self.add_parameter('y', 1.0)
		self.add_parameter('theta', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:699 y:30, x:496 y:139
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:33 y:30
			OperatableStateMachine.add('WaitForGo',
										WaitForNaoQiSpeechState(string_to_rec="go"),
										transitions={'done': 'WaitForOpenDoor'},
										autonomy={'done': Autonomy.Off})

			# x:235 y:130
			OperatableStateMachine.add('NavigateToGoal',
										MoveBaseState(),
										transitions={'arrived': 'SayHi', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:234 y:21
			OperatableStateMachine.add('SetNavGoal',
										GenerateNavgoalState(x=self.x, y=self.y, theta=self.theta),
										transitions={'done': 'NavigateToGoal'},
										autonomy={'done': Autonomy.Off},
										remapping={'navgoal': 'waypoint'})

			# x:454 y:24
			OperatableStateMachine.add('SayHi',
										TalkState(message="Hi, I am here", blocking=True),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:44 y:132
			OperatableStateMachine.add('WaitForOpenDoor',
										WaitForOpenDoorState(),
										transitions={'done': 'SetNavGoal'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
