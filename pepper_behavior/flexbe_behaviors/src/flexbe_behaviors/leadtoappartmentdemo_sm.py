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
from pepper_flexbe_states.generate_navgoal import GenerateNavGoalState
from pepper_flexbe_states.talk_state import TalkState
from pepper_flexbe_states.check_for_person_state import CheckForPersonState
from pepper_flexbe_states.wait import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jan 17 2018
@author: ffriese
'''
class LeadToAppartmentDemoSM(Behavior):
    '''
    Simple Demo where Pepper leads Guests to the Appartment
    '''


    def __init__(self):
        super(LeadToAppartmentDemoSM, self).__init__()
        self.name = 'LeadToAppartmentDemo'

        # parameters of this behavior

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:766 y:486, x:382 y:306
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:78 y:28
            OperatableStateMachine.add('WaitForGuideCommand',
                                        WaitForNaoQiSpeechState(strings_to_rec=['Hallo Pepper bitte bring unseren Gast zum Appartment'], outcomes=['appartment'], topic='/pepper_robot/speechrec/context'),
                                        transitions={'appartment': 'AcknowlegdeCommand'},
                                        autonomy={'appartment': Autonomy.Off})

            # x:329 y:147
            OperatableStateMachine.add('GuideToAppartment',
                                        MoveBaseState(head_angle=0),
                                        transitions={'arrived': 'AnnounceArrival', 'failed': 'AnnounceFailure'},
                                        autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'waypoint': 'outside_appartment'})

            # x:641 y:25
            OperatableStateMachine.add('SetNavgoalAtAppartment',
                                        GenerateNavGoalState(x=14.3982963562, y=-4.98356437683, theta=-105.676, frame_id='map'),
                                        transitions={'done': 'GuideToAppartment'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'generated_goal': 'outside_appartment'})

            # x:106 y:157
            OperatableStateMachine.add('AnnounceArrival',
                                        TalkState(message='Wir sind angekommen. Bitte treten Sie ein, ich komme nach.', blocking=True),
                                        transitions={'done': 'WaitForClear', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:546 y:232
            OperatableStateMachine.add('AnnounceFailure',
                                        TalkState(message='Leider konnte ich meine Aufgabe nicht ausführen. Es tut mir sehr leid.', blocking=True),
                                        transitions={'done': 'failed', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:103 y:319
            OperatableStateMachine.add('SetNavgoalInside',
                                        GenerateNavGoalState(x=20.5, y=-5.7, theta=-110, frame_id='map'),
                                        transitions={'done': 'DriveInside'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'generated_goal': 'inside_appartment'})

            # x:106 y:412
            OperatableStateMachine.add('DriveInside',
                                        MoveBaseState(head_angle=0),
                                        transitions={'arrived': 'SearchForBill', 'failed': 'failed'},
                                        autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'waypoint': 'inside_appartment'})

            # x:342 y:26
            OperatableStateMachine.add('AcknowlegdeCommand',
                                        TalkState(message='Mit Vergnügen. Bitte folgen Sie mir.', blocking=True),
                                        transitions={'done': 'SetNavgoalAtAppartment', 'failed': 'SetNavgoalAtAppartment'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:327 y:439
            OperatableStateMachine.add('SearchForBill',
                                        CheckForPersonState(name='billexample', timeout=2.0),
                                        transitions={'found': 'SayHello', 'not_found': 'SayNotFound', 'failed': 'failed'},
                                        autonomy={'found': Autonomy.Off, 'not_found': Autonomy.Off, 'failed': Autonomy.Off})

            # x:552 y:499
            OperatableStateMachine.add('SayHello',
                                        TalkState(message='Hallo Bill!', blocking=True),
                                        transitions={'done': 'finished', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:593 y:359
            OperatableStateMachine.add('SayNotFound',
                                        TalkState(message='Ich konnte Bill leider nicht finden.', blocking=True),
                                        transitions={'done': 'finished', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:103 y:231
            OperatableStateMachine.add('WaitForClear',
                                        WaitState(time=30),
                                        transitions={'done': 'SetNavgoalInside'},
                                        autonomy={'done': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
