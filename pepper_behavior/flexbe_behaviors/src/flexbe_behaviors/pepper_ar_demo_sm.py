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
from generic_flexbe_states.publisher_bool_state import PublisherBoolState
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
        # x:143 y:317, x:152 y:208
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.True = True
        _state_machine.userdata.False = False

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:44 y:50
            OperatableStateMachine.add('WaitForGraspCommand',
                                        WaitForNaoQiSpeechState(strings_to_rec=['Grasp here'], outcomes=['grasp'], topic='/pepper_robot/speechrec/context'),
                                        transitions={'grasp': 'SayPlace'},
                                        autonomy={'grasp': Autonomy.Off})

            # x:501 y:58
            OperatableStateMachine.add('GraspSpaceOn',
                                        PublisherBoolState(topic='/pepper_robot/hololens/grasp_space'),
                                        transitions={'done': 'WaitForObject'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'value': 'True'})

            # x:725 y:254
            OperatableStateMachine.add('GraspSpaceOff',
                                        PublisherBoolState(topic='/pepper_robot/hololens/grasp_space'),
                                        transitions={'done': 'GRASP'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'value': 'False'})

            # x:289 y:58
            OperatableStateMachine.add('SayPlace',
                                        TalkState(message='Please place the object in the highlighted space', blocking=True),
                                        transitions={'done': 'GraspSpaceOn', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:708 y:58
            OperatableStateMachine.add('WaitForObject',
                                        WaitForNaoQiSpeechState(strings_to_rec=['There you go'], outcomes=['done'], topic='/pepper_robot/speechrec/context'),
                                        transitions={'done': 'GraspSpaceOff'},
                                        autonomy={'done': Autonomy.Off})

            # x:410 y:249
            OperatableStateMachine.add('GRASP',
                                        TalkState(message='I would be grasping now, if I could', blocking=True),
                                        transitions={'done': 'finished', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
