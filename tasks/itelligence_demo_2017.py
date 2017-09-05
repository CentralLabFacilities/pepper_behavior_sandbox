#!/usr/bin/env python

import rospy
import smach
import smach_ros

from pepper_behavior.skills.animation_player import AnimationPlayerPepper
from pepper_behavior.skills.calculate_person_position import CalculatePersonPosition
from pepper_behavior.skills.iterate import Counter, Iterate
from pepper_behavior.skills.move_head import MoveHeadPepper
from pepper_behavior.skills.talk import Talk

from pepper_behavior.actuators.head_control import HeadControlPepper
from pepper_behavior.actuators.talk import TalkControllerPepper

from pepper_behavior.sensors.person_sensor import PersonSensor


def main():
    simulation = False
    rospy.init_node('intelligence_pepper_state_machine')
    hc = HeadControlPepper()
    tc = TalkControllerPepper(sim=simulation)
    ps = PersonSensor()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.mode = 0

    wait_timer_idle = 15
    wait_timer_attention = 10
    look_vertical = 'up'

    # Open the container
    with sm:
        smach.StateMachine.add(
            'Iterate', Iterate(iterationsteps=3),
            transitions={'success_0': 'Idle_Statemaschine', 'success_1': 'Attention_Statemaschine',
                         'success_2': 'Look_Statemaschine'},
            remapping={'iterate_input': 'mode', 'iterate_output': 'mode'})

        sm_idle = smach.StateMachine(outcomes=['idle_success'])
        sm_attention = smach.StateMachine(outcomes=['attention_success'])
        sm_look = smach.StateMachine(outcomes=['look_success'])

        with sm_idle:
            sm_idle.userdata.iteration = 0

            smach.StateMachine.add(
                'Talk_mode_idle', Talk(controller=tc, text='Idle Mode'),
                transitions={'success': 'Counter_idle'})

            smach.StateMachine.add(
                'Counter_idle', Counter(numbers=1),
                transitions={'success': 'MoveHead_left_idle', 'end': 'idle_success'},
                remapping={'counter_input': 'iteration', 'counter_output': 'iteration'})

            smach.StateMachine.add(
                'MoveHead_left_idle',
                MoveHeadPepper(_hv=look_vertical, _hh='left', controller=hc, wait=wait_timer_idle),
                transitions={'success': 'MoveHead_center_idle'})

            smach.StateMachine.add(
                'MoveHead_center_idle',
                MoveHeadPepper(_hv=look_vertical, _hh='center', controller=hc, wait=wait_timer_idle),
                transitions={'success': 'MoveHead_right_idle'})

            smach.StateMachine.add(
                'MoveHead_right_idle',
                MoveHeadPepper(_hv=look_vertical, _hh='right', controller=hc, wait=wait_timer_idle),
                transitions={'success': 'MoveHead_center_idle_2'})

            smach.StateMachine.add(
                'MoveHead_center_idle_2',
                MoveHeadPepper(_hv=look_vertical, _hh='center', controller=hc, wait=wait_timer_idle),
                transitions={'success': 'Counter_idle'})

        smach.StateMachine.add('Idle_Statemaschine', sm_idle, transitions={'idle_success': 'Iterate'})

        with sm_attention:
            sm_attention.userdata.iteration = 1
            sm_attention.userdata.vertical_angle = 0.0
            sm_attention.userdata.horizontal_angle = 0.0

            smach.StateMachine.add(
                'Iterate', Iterate(iterationsteps=4),
                transitions={'success_0': 'attention_success', 'success_1': 'MoveHead_left',
                             'success_2': 'MoveHead_center', 'success_3': 'MoveHead_right'},
                remapping={'iterate_input': 'iteration', 'iterate_output': 'iteration'})

            smach.StateMachine.add(
                'MoveHead_left',
                MoveHeadPepper(_hv=look_vertical, _hh='left', controller=hc, wait=wait_timer_attention),
                transitions={'success': 'CalculatePersonPosition'})

            smach.StateMachine.add(
                'MoveHead_center',
                MoveHeadPepper(_hv=look_vertical, _hh='center', controller=hc, wait=wait_timer_attention),
                transitions={'success': 'CalculatePersonPosition'})

            smach.StateMachine.add(
                'MoveHead_right',
                MoveHeadPepper(_hv=look_vertical, _hh='right', controller=hc, wait=wait_timer_attention),
                transitions={'success': 'CalculatePersonPosition'})

            smach.StateMachine.add(
                'CalculatePersonPosition', CalculatePersonPosition(controller=ps, max_distance=2.5),
                transitions={'success': 'LookToPerson', 'repeat': 'CalculatePersonPosition',
                             'no_person_found': 'Iterate'},
                remapping={'person_angle_vertical': 'vertical_angle', 'person_angle_horizontal': 'horizontal_angle'})

            smach.StateMachine.add(
                'LookToPerson', MoveHeadPepper(controller=hc),
                transitions={'success': 'Animation'},
                remapping={'head_vertical': 'vertical_angle', 'head_horizontal': 'horizontal_angle'})

            smach.StateMachine.add(
                'Animation', AnimationPlayerPepper(animation='animations/Stand/Emotions/Positive/Happy_1'),
                transitions={'success': 'TalkWelcome'})

            smach.StateMachine.add(
                'TalkWelcome', Talk(controller=tc, text='Hallo, ich bin Pepper!'),  # Herzlich willkommen auf der '
                # 'itelligence World 2017! Ich bin ein humanoider Roboter '
                # 'und arbeite zur Zeit am CITEC der Universitaet Bielefeld.', wait=20),
                transitions={'success': 'MoveHead_demo'})

            smach.StateMachine.add(
                'MoveHead_demo',
                MoveHeadPepper(_hv=look_vertical, _hh='right', controller=hc, wait=3),
                transitions={'success': 'Animation_demo'})

            smach.StateMachine.add(
                'Animation_demo', AnimationPlayerPepper(animation='animations/Stand/Emotions/Positive/Happy_1'),
                transitions={'success': 'TalkWelcome_demo'})

            smach.StateMachine.add(
                'TalkWelcome_demo', Talk(controller=tc, text='Wir haben dieses'),
                # Jahr wieder eine Menge Demoszenarien zu '
                # 'Industrie 4.0 und Internet of Things Werfen Sie nach der
                # 'Key-Note doch einfach mal einen Blick in unsere '
                # 'Ausstellung.', wait=20),
                transitions={'success': 'Iterate'})

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