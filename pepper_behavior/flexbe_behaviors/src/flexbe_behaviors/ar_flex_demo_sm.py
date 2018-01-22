#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_behaviors.pepper_ar_demo_sm import PepperARDemoSM
from pepper_flexbe_states.wait_for_ros_navgoal import WaitForRosNavgoalState
from pepper_flexbe_states.set_navgoal_state import MoveBaseState
from pepper_flexbe_states.talk_state import TalkState
from pepper_flexbe_states.wait_for_naoqi_speech import WaitForNaoQiSpeechState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jan 22 2018
@author: ffriese
'''
class AR_Flex_DemoSM(Behavior):
    '''
    Flexible AR-Demo
    '''


    def __init__(self):
        super(AR_Flex_DemoSM, self).__init__()
        self.name = 'AR_Flex_Demo'

        # parameters of this behavior

        # references to used behaviors
        self.add_behavior(PepperARDemoSM, 'InterruptibleDemo/Pepper AR Demo')

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:486 y:179, x:68 y:193
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]

        # x:417 y:37, x:130 y:365
        _sm_movetogoal_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['goal'])

        with _sm_movetogoal_0:
            # x:88 y:28
            OperatableStateMachine.add('MoveToHololens',
                                        MoveBaseState(head_angle=25),
                                        transitions={'arrived': 'finished', 'failed': 'SayBlocked'},
                                        autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'waypoint': 'goal'})

            # x:347 y:142
            OperatableStateMachine.add('SayBlocked',
                                        TalkState(message='The way is blocked. Please clear the way and tell me when it is clear.', blocking=True),
                                        transitions={'done': 'WaitForClear', 'failed': 'WaitForClear'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:80 y:247
            OperatableStateMachine.add('WaitForClear',
                                        WaitForNaoQiSpeechState(strings_to_rec=['The way is clear now'], outcomes=['clear'], topic='/pepper_robot/speechrec/context'),
                                        transitions={'clear': 'MoveToHololens'},
                                        autonomy={'clear': Autonomy.Off})


        # x:533 y:33, x:115 y:357, x:540 y:161, x:537 y:83, x:39 y:358, x:187 y:354
        _sm_interruptiblemove_1 = ConcurrencyContainer(outcomes=['finished', 'failed', 'interrupted'], input_keys=['goal'], output_keys=['hololens_gen'], conditions=[
                                        ('finished', [('MoveToGoal', 'finished')]),
                                        ('finished', [('MoveToGoal', 'failed')]),
                                        ('interrupted', [('WaitForHololensGoal', 'done')])
                                        ])

        with _sm_interruptiblemove_1:
            # x:151 y:34
            OperatableStateMachine.add('MoveToGoal',
                                        _sm_movetogoal_0,
                                        transitions={'finished': 'finished', 'failed': 'finished'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'goal': 'goal'})

            # x:140 y:144
            OperatableStateMachine.add('WaitForHololensGoal',
                                        WaitForRosNavgoalState(topic='/hololens/navgoal'),
                                        transitions={'done': 'interrupted'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'navgoal': 'hololens_gen'})


        # x:471 y:22, x:130 y:365, x:447 y:167, x:470 y:102, x:206 y:368, x:276 y:363
        _sm_interruptibledemo_2 = ConcurrencyContainer(outcomes=['finished', 'failed', 'interrupted'], output_keys=['hololens_gen'], conditions=[
                                        ('interrupted', [('WaitForHololensGoal', 'done')]),
                                        ('finished', [('Pepper AR Demo', 'finished')]),
                                        ('finished', [('Pepper AR Demo', 'failed')])
                                        ])

        with _sm_interruptibledemo_2:
            # x:119 y:28
            OperatableStateMachine.add('Pepper AR Demo',
                                        self.use_behavior(PepperARDemoSM, 'InterruptibleDemo/Pepper AR Demo'),
                                        transitions={'finished': 'finished', 'failed': 'finished'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:121 y:156
            OperatableStateMachine.add('WaitForHololensGoal',
                                        WaitForRosNavgoalState(topic='/hololens/navgoal'),
                                        transitions={'done': 'interrupted'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'navgoal': 'hololens_gen'})



        with _state_machine:
            # x:262 y:23
            OperatableStateMachine.add('InterruptibleDemo',
                                        _sm_interruptibledemo_2,
                                        transitions={'finished': 'InterruptibleDemo', 'failed': 'failed', 'interrupted': 'InterruptibleMove'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'interrupted': Autonomy.Inherit},
                                        remapping={'hololens_gen': 'hololens_gen'})

            # x:260 y:281
            OperatableStateMachine.add('InterruptibleMove',
                                        _sm_interruptiblemove_1,
                                        transitions={'finished': 'InterruptibleDemo', 'failed': 'failed', 'interrupted': 'InterruptibleMove'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'interrupted': Autonomy.Inherit},
                                        remapping={'goal': 'hololens_gen', 'hololens_gen': 'hololens_gen'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
