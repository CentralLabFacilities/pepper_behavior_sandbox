#!/usr/bin/env python

import rospy
import smach
import smach_ros
from actuators.base_control import BaseControlPepper
from actuators.head_control import HeadControlPepper
from skills.move_head import MoveHeadPepper
from skills.turn_base import TurnBasePepper
from actuators.talk import TalkControllerPepper
from skills.talk import Talk
from skills.iterate import Counter, Iterate
from skills.animation_player import AnimationPlayerPepper

def main():
    simulation = True
    rospy.init_node('intelligence_pepper_state_machine')
    hc = HeadControlPepper()
    bc = BaseControlPepper()
    tc = TalkControllerPepper(sim=simulation)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.mode = 0

    wait_timer_idle = 15
    look_vertical = 'down'


    # Open the container
    with sm:

        smach.StateMachine.add(
            'Animation', AnimationPlayerPepper(scope='/pepper/animation_player', animation='animations/Stand/Emotions/Positive/Happy_1'),
            transitions={'success': 'Animation'})

        smach.StateMachine.add(
            'Iterate', Iterate(iterationsteps=3),
            transitions={'success_0':'Idle_Statemaschine','success_1':'Attention_Statemaschine', 'success_2':'Look_Statemaschine'},
            remapping={'iterate_input':'mode', 'iterate_output':'mode'})

        sm_idle = smach.StateMachine(outcomes=['idle_success'])
        sm_attention = smach.StateMachine(outcomes=['attention_success'])
        sm_look = smach.StateMachine(outcomes=['look_success'])

        with sm_idle:
            sm_idle.userdata.iteration = 0

            smach.StateMachine.add(
                'Talk_mode_idle', Talk(controller=tc, text='Idle Mode'),
                transitions={'success': 'Counter_idle'})

            smach.StateMachine.add(
                'Counter_idle', Counter(numbers=3),
                transitions={'success':'MoveHead_left_idle', 'end':'idle_success'},
                remapping={'counter_input': 'iteration', 'counter_output': 'iteration'})

            smach.StateMachine.add(
                'MoveHead_left_idle', MoveHeadPepper(_hv=look_vertical, _hh='left', controller=hc, wait=wait_timer_idle),
                transitions={'success':'MoveHead_center_idle'})

            smach.StateMachine.add(
                'MoveHead_center_idle', MoveHeadPepper(_hv=look_vertical, _hh='center', controller=hc, wait=wait_timer_idle),
                transitions={'success': 'MoveHead_right_idle'})

            smach.StateMachine.add(
                'MoveHead_right_idle', MoveHeadPepper(_hv=look_vertical, _hh='right', controller=hc, wait=wait_timer_idle),
                transitions={'success': 'MoveHead_center_idle_2'})

            smach.StateMachine.add(
                'MoveHead_center_idle_2', MoveHeadPepper(_hv=look_vertical, _hh='center', controller=hc, wait=wait_timer_idle),
                transitions={'success': 'Counter_idle'})

        smach.StateMachine.add('Idle_Statemaschine', sm_idle, transitions={'idle_success': 'Iterate'})

        with sm_attention:
            sm_attention.userdata.iteration = 0

            smach.StateMachine.add(
                'Talk_mode_attention', Talk(controller=tc, text='Attention Mode'),
                transitions={'success': 'Counter_attention'})

            smach.StateMachine.add(
                'Counter_attention', Counter(numbers=3),
                transitions={'end': 'attention_success', 'success': 'Talk_mode_attention'},
                remapping={'counter_input': 'iteration', 'counter_output': 'iteration'})

        smach.StateMachine.add('Attention_Statemaschine', sm_attention, transitions={'attention_success': 'Iterate'})

        with sm_look:
            sm_look.userdata.iteration = 0

            smach.StateMachine.add(
                'Talk_mode_look_person', Talk(controller=tc, text='Look Mode'),
                transitions={'success': 'Counter_look'})

            smach.StateMachine.add(
                'Counter_look', Counter(numbers=3),
                transitions={'end': 'look_success', 'success': 'Talk_mode_look_person'},
                remapping={'counter_input': 'iteration', 'counter_output': 'iteration'})

        smach.StateMachine.add('Look_Statemaschine', sm_look, transitions={'look_success': 'Iterate'})

    # Introspection viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/GENIALE')
    sis.start()

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    main()
