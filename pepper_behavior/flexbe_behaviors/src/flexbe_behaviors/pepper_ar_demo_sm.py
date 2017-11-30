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
from flexbe_states.publisher_bool_state import PublisherBoolState
from pepper_flexbe_states.wait_for_naoqi_speech import WaitForNaoQiSpeechState
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
		self.add_parameter('ForceRealRobot', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1090 y:346, x:732 y:202
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.True = True
		_state_machine.userdata.False = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:28 y:334
			OperatableStateMachine.add('WaitForHololensGoal',
										WaitForRosNavgoalState(topic='/hololens/navgoal'),
										transitions={'done': 'DriveToHololensGoal'},
										autonomy={'done': Autonomy.Off},
										remapping={'navgoal': 'navgoal'})

			# x:46 y:57
			OperatableStateMachine.add('DriveToHololensGoal',
										MoveBaseState(),
										transitions={'arrived': 'SayHereIAm', 'failed': 'SayWayBlocked'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'navgoal'})

			# x:324 y:189
			OperatableStateMachine.add('SayWayBlocked',
										TalkState(message='My way is blocked. Please clear the way and call me again', blocking=True, force_real_robot=self.ForceRealRobot),
										transitions={'done': 'WaitForWayClear', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:645 y:56
			OperatableStateMachine.add('GraspSpaceOn',
										PublisherBoolState(topic='/pepper_robot/hololens/grasp_space'),
										transitions={'done': 'WaitForObject'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'True'})

			# x:1042 y:56
			OperatableStateMachine.add('GraspSpaceOff',
										PublisherBoolState(topic='/pepper_robot/hololens/grasp_space'),
										transitions={'done': 'GRASP'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'False'})

			# x:335 y:56
			OperatableStateMachine.add('SayHereIAm',
										TalkState(message='Here I am, please place the object in the highlighted space', blocking=True, force_real_robot=self.ForceRealRobot),
										transitions={'done': 'GraspSpaceOn', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:308 y:332
			OperatableStateMachine.add('WaitForWayClear',
										WaitForNaoQiSpeechState(strings_to_rec=['The way is clear now'], outcomes=['clear'], topic='/pepper_robot/speechrec/context'),
										transitions={'clear': 'WaitForHololensGoal'},
										autonomy={'clear': Autonomy.Off})

			# x:827 y:53
			OperatableStateMachine.add('WaitForObject',
										WaitForNaoQiSpeechState(strings_to_rec=['There you go'], outcomes=['done'], topic='/pepper_robot/speechrec/context'),
										transitions={'done': 'GraspSpaceOff'},
										autonomy={'done': Autonomy.Off})

			# x:1036 y:182
			OperatableStateMachine.add('GRASP',
										TalkState(message='I would be grasping now, if I could', blocking=True, force_real_robot=self.ForceRealRobot),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
