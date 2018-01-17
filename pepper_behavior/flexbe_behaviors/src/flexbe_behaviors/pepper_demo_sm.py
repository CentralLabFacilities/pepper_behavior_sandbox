#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from pepper_flexbe_states.wait_for_open_door import WaitForOpenDoorState
from pepper_flexbe_states.generate_navgoal import GenerateNavGoalState
from pepper_flexbe_states.talk_state import TalkState
from pepper_flexbe_states.wait_for_naoqi_speech import WaitForNaoQiSpeechState
from pepper_flexbe_states.set_navgoal_state import MoveBaseState
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
		self.add_parameter('x', 3.75)
		self.add_parameter('y', 4.62)
		self.add_parameter('theta', 0)

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
			# x:33 y:26
			OperatableStateMachine.add('WaitForOpenDoor',
										WaitForOpenDoorState(),
										transitions={'done': 'SetNavGoalInside'},
										autonomy={'done': Autonomy.Off})

			# x:224 y:26
			OperatableStateMachine.add('SetNavGoalInside',
										GenerateNavGoalState(x=self.x, y=self.y, theta=self.theta, frame_id='map'),
										transitions={'done': 'NavigateToOrderingPos'},
										autonomy={'done': Autonomy.Off},
										remapping={'generated_goal': 'waypoint'})

			# x:454 y:24
			OperatableStateMachine.add('SayHiAndAskForCommand',
										TalkState(message="Hi, I am here. How can I help you?", blocking=True),
										transitions={'done': 'WaitForOrder', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:723 y:13
			OperatableStateMachine.add('WaitForOrder',
										WaitForNaoQiSpeechState(strings_to_rec=['Do you know me','What is this'], outcomes=['know','what'], topic='/pepper_robot/speechrec/context'),
										transitions={'know': 'SayDontKnow', 'what': 'SayNoIdea'},
										autonomy={'know': Autonomy.Off, 'what': Autonomy.Off})

			# x:692 y:121
			OperatableStateMachine.add('SayDontKnow',
										TalkState(message='No, I dont know you', blocking=True),
										transitions={'done': 'SetNavGoalStartPos', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:867 y:165
			OperatableStateMachine.add('SayNoIdea',
										TalkState(message='I have no Idea', blocking=True),
										transitions={'done': 'SetNavGoalStartPos', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:826 y:326
			OperatableStateMachine.add('SetNavGoalStartPos',
										GenerateNavGoalState(x=0, y=0, theta=0.0, frame_id='odom'),
										transitions={'done': 'NavigateToStartPos'},
										autonomy={'done': Autonomy.Off},
										remapping={'generated_goal': 'waypoint2'})

			# x:434 y:342
			OperatableStateMachine.add('NavigateToStartPos',
										MoveBaseState(head_angle=25),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint2'})

			# x:218 y:116
			OperatableStateMachine.add('NavigateToOrderingPos',
										MoveBaseState(head_angle=25),
										transitions={'arrived': 'SayHiAndAskForCommand', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
