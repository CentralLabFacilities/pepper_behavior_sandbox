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
        self.add_behavior(PepperARDemoSM, 'Container/Container/Pepper AR Demo')

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:30 y:365, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]

        # x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
        _sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'override'], output_keys=['navgoal'], conditions=[
                                        ('finished', [('Pepper AR Demo', 'finished')]),
                                        ('override', [('OverrideWithHololensGoal', 'done')]),
                                        ('failed', [('Pepper AR Demo', 'failed')])
                                        ])

        with _sm_container_0:
            # x:23 y:86
            OperatableStateMachine.add('Pepper AR Demo',
                                        self.use_behavior(PepperARDemoSM, 'Container/Container/Pepper AR Demo'),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:201 y:85
            OperatableStateMachine.add('OverrideWithHololensGoal',
                                        WaitForRosNavgoalState(topic='/hololens/navgoal'),
                                        transitions={'done': 'override'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'navgoal': 'navgoal'})


        # x:30 y:365, x:130 y:365
        _sm_container_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _sm_container_1:
            # x:119 y:79
            OperatableStateMachine.add('Container',
                                        _sm_container_0,
                                        transitions={'finished': 'Container', 'failed': 'failed', 'override': 'DriveToGoal'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'override': Autonomy.Inherit},
                                        remapping={'navgoal': 'navgoal'})

            # x:540 y:26
            OperatableStateMachine.add('DriveToGoal',
                                        MoveBaseState(head_angle=25),
                                        transitions={'arrived': 'Container', 'failed': 'Container'},
                                        autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'waypoint': 'navgoal'})



        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('Container',
                                        _sm_container_1,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
