#!/usr/bin/python2

import rospy
import smach
import smach_ros

from std_msgs.msg import Float64

from pepper_behavior.skills.animation_player import AnimationPlayerPepper
from pepper_behavior.skills.calculate_person_position import CalculatePersonPosition
from pepper_behavior.skills.iterate import Counter, Iterate
from pepper_behavior.skills.move_head import MoveHeadPepper
from pepper_behavior.skills.talk import Talk
from pepper_behavior.skills.ssl import Ssl
from pepper_behavior.skills.point import LeftArmGesture
from pepper_behavior.skills.state_publisher import StatePublisher
from pepper_behavior.skills.turn_base_without import TurnWithoutMovebase

from pepper_behavior.actuators.head_control import HeadControlPepper
from pepper_behavior.actuators.talk import TalkControllerPepper
from pepper_behavior.actuators.ros_string_pub import RosStringPub
from pepper_behavior.actuators.arm_control import LeftArmControlPepper


from pepper_behavior.sensors.person_sensor import PersonSensor
from pepper_behavior.sensors.ros_sub import RosSub


def main():
    simulation = False
    rospy.init_node('intelligence_pepper_state_machine')
    hc = HeadControlPepper()
    tc = TalkControllerPepper(sim=simulation)
    ps = PersonSensor()
    lac = LeftArmControlPepper()
    rs_ssl = RosSub(dataType=Float64, scope='/pepper_robot/ssl/angle')
    animation_pub = RosStringPub('/pepper_robot/animation_player')
    st = RosStringPub('/pepper_robot/smach/state')
    cc = RosStringPub('/pepper_robot/leds')
    turn_pub = RosStringPub('/pepper_robot/drivedirect')

    rospy.sleep(1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.mode = 0

    wait_timer_idle = 6
    wait_timer_attention = 3
    look_vertical = 'drive'

    # Open the container
    with sm:
        smach.StateMachine.add(
            'Init_Talk', Talk(controller=tc, text='Demo startet.'), transitions={'success': 'Init_state'})

        smach.StateMachine.add('Init_state', StatePublisher(st,'init'),transitions={'success': 'Iterate'})

        smach.StateMachine.add(
            'Iterate', Iterate(iterationsteps=3),
            transitions={'success_0': 'Idle_Statemaschine', 'success_1': 'Attention_Statemaschine',
                         'success_2': 'Idle_Statemaschine'},
            remapping={'iterate_input': 'mode', 'iterate_output': 'mode'})

        sm_idle = smach.StateMachine(outcomes=['idle_success'])
        sm_attention = smach.StateMachine(outcomes=['attention_success'])
        # sm_look = smach.StateMachine(outcomes=['look_success'])

        with sm_idle:
            sm_idle.userdata.iteration = 0
            sm_idle.userdata.animationiterator = 0

            smach.StateMachine.add('idle_state', StatePublisher(st, 'idle_mode',colorcontroller=cc,color='FaceLeds:yellow'), transitions={'success': 'Counter_idle'})

            smach.StateMachine.add(
                'Counter_idle', Counter(numbers=1),
                transitions={'success': 'MoveHead_left_idle', 'end': 'idle_success'},
                remapping={'counter_input': 'iteration', 'counter_output': 'iteration'})


            smach.StateMachine.add(
                'MoveHead_left_idle',
                MoveHeadPepper(_hv=look_vertical, _hh='left', controller=hc, wait=wait_timer_idle,speed=0.1),
                transitions={'success': 'Counter_animation_idle'})

            smach.StateMachine.add(
                'Counter_animation_idle', Counter(numbers=2),
                transitions={'success': 'Animation_idle', 'end': 'Animation_idle'},
                remapping={'counter_input': 'animationiterator', 'counter_output': 'animationiterator'})

            smach.StateMachine.add(
                'Animation_idle',
                AnimationPlayerPepper(controller=animation_pub, animationblock='idle'),
                transitions={'success': 'MoveHead_center_idle'}, remapping={'id': 'animationiterator'})

            smach.StateMachine.add(
                'MoveHead_center_idle',
                MoveHeadPepper(_hv=look_vertical, _hh='center', controller=hc, wait=wait_timer_idle,speed=0.1),
                transitions={'success': 'Counter_animation_idle_2'})

            smach.StateMachine.add(
                'Counter_animation_idle_2', Counter(numbers=2),
                transitions={'success': 'Animation_idle_2', 'end': 'Animation_idle_2'},
                remapping={'counter_input': 'animationiterator', 'counter_output': 'animationiterator'})

            smach.StateMachine.add(
                'Animation_idle_2',
                AnimationPlayerPepper(controller=animation_pub, animationblock='idle'),
                transitions={'success': 'MoveHead_right_idle'}, remapping={'id': 'animationiterator'})

            smach.StateMachine.add(
                'MoveHead_right_idle',
                MoveHeadPepper(_hv=look_vertical, _hh='right', controller=hc, wait=wait_timer_idle,speed=0.1),
                transitions={'success': 'Counter_animation_idle_3'})

            smach.StateMachine.add(
                'Counter_animation_idle_3', Counter(numbers=2),
                transitions={'success': 'Animation_idle_3', 'end': 'Animation_idle_3'},
                remapping={'counter_input': 'animationiterator', 'counter_output': 'animationiterator'})

            smach.StateMachine.add(
                'Animation_idle_3',
                AnimationPlayerPepper(controller=animation_pub, animationblock='idle'),
                transitions={'success': 'MoveHead_center_idle_2'}, remapping={'id': 'animationiterator'})

            smach.StateMachine.add(
                'MoveHead_center_idle_2',
                MoveHeadPepper(_hv=look_vertical, _hh='center', controller=hc, wait=wait_timer_idle),
                transitions={'success': 'Counter_idle'})

        smach.StateMachine.add('Idle_Statemaschine', sm_idle, transitions={'idle_success': 'Iterate'})

        with sm_attention:
            sm_attention.userdata.iteration = 0
            sm_attention.userdata.vertical_angle = 0.0
            sm_attention.userdata.horizontal_angle = 0.0
            sm_attention.userdata.iterationtext = 0

            smach.StateMachine.add('attention_state', StatePublisher(st, 'attention_mode',colorcontroller=cc,color='FaceLeds:cyan'), transitions={'success': 'Iterate'})

            smach.StateMachine.add(
                'Iterate', Iterate(iterationsteps=4),
                transitions={'success_0': 'MoveBase_end', 'success_1': 'MoveBase_left',
                             'success_2': 'MoveBase_center', 'success_3': 'MoveBase_right'},
                remapping={'iterate_input': 'iteration', 'iterate_output': 'iteration'})

            smach.StateMachine.add(
                'MoveBase_end',
                TurnWithoutMovebase( controller=turn_pub, angle=25),
                transitions={'success': 'attention_success'})

            smach.StateMachine.add(
                'MoveBase_left',
                TurnWithoutMovebase( controller=turn_pub, angle=25),
                transitions={'success': 'MoveHead_left'})

            smach.StateMachine.add(
                'MoveBase_center',
                TurnWithoutMovebase( controller=turn_pub, angle=-25),
                transitions={'success': 'MoveHead_center'})

            smach.StateMachine.add(
                'MoveBase_right',
                TurnWithoutMovebase( controller=turn_pub, angle=-25),
                transitions={'success': 'MoveHead_right'})

            smach.StateMachine.add(
                'MoveHead_left',
                MoveHeadPepper(_hv='up', _hh='left', controller=hc, wait=4, speed=0.1),
                transitions={'success': 'CalculatePersonPosition'})

            smach.StateMachine.add(
                'MoveHead_center',
                MoveHeadPepper(_hv='up', _hh='center', controller=hc, wait=wait_timer_attention, speed=0.1),
                transitions={'success': 'CalculatePersonPosition'})

            smach.StateMachine.add(
                'MoveHead_right',
                MoveHeadPepper(_hv='up', _hh='right', controller=hc, wait=wait_timer_attention, speed=0.1),
                transitions={'success': 'CalculatePersonPosition'})

            smach.StateMachine.add(
                'CalculatePersonPosition', CalculatePersonPosition(controller=ps, max_distance=2.5),
                transitions={'success': 'Counter_text', 'repeat': 'CalculatePersonPosition',
                             'no_person_found': 'Iterate'},
                remapping={'person_angle_vertical': 'vertical_angle', 'person_angle_horizontal': 'horizontal_angle'})

            smach.StateMachine.add(
                'Counter_text', Counter(numbers=4),
                transitions={'success': 'LookToPerson', 'end': 'LookToPerson'},
                remapping={'counter_input': 'iterationtext', 'counter_output': 'iterationtext'})

            smach.StateMachine.add(
                'LookToPerson', MoveHeadPepper(controller=hc, wait=1.5, speed=0.1),
                transitions={'success': 'Animation'},
                remapping={'head_vertical': 'vertical_angle', 'head_horizontal': 'horizontal_angle'})

            smach.StateMachine.add(
                'Animation',
                AnimationPlayerPepper(controller=animation_pub),
                transitions={'success': 'TalkWelcome'}, remapping={'id': 'iterationtext'})

            smach.StateMachine.add(
                'TalkWelcome', Talk(controller=tc),
                transitions={'success': 'CalculatePersonPosition_recalc'}, remapping={'id': 'iterationtext'})

            smach.StateMachine.add(
                'CalculatePersonPosition_recalc', CalculatePersonPosition(controller=ps, max_distance=2.5),
                transitions={'success': 'LookToPerson_afterAnimation', 'repeat': 'LookToPerson_afterAnimation',
                             'no_person_found': 'LookToPerson_afterAnimation'},
                remapping={'person_angle_vertical': 'vertical_angle', 'person_angle_horizontal': 'horizontal_angle'})

            smach.StateMachine.add(
                'LookToPerson_afterAnimation', MoveHeadPepper(controller=hc, wait=1, speed=0.1),
                transitions={'success': 'Animation_talking'},
                remapping={'head_vertical': 'vertical_angle', 'head_horizontal': 'horizontal_angle'})

            smach.StateMachine.add(
                'Animation_talking',
                AnimationPlayerPepper(controller=animation_pub, animationblock='talking'),
                transitions={'success': 'TalkWelcome_demo'}, remapping={'id': 'iterationtext'})


            smach.StateMachine.add(
                'TalkWelcome_demo', Talk(controller=tc, textblock='explain'),
                transitions={'success': 'LookToPoint'}, remapping={'id': 'iterationtext'})

            smach.StateMachine.add(
                'LookToPoint', MoveHeadPepper(controller=hc, _hh='right', _hv='up', wait=0,speed=0.1),
                transitions={'success': 'Point_demo'})

            smach.StateMachine.add(
                'Point_demo', LeftArmGesture(controller=lac, gesture='demo', wait=3),
                transitions={'success': 'Iterate',
                             'unknown_gesture': 'Iterate'})

        smach.StateMachine.add('Attention_Statemaschine', sm_attention, transitions={'attention_success': 'Iterate'})

        # with sm_look:
        #     sm_look.userdata.iteration = 0
        #     sm_look.userdata.horizontal_direction = 0.0
        #
        #     smach.StateMachine.add('look_state', StatePublisher(st, 'look_mode',colorcontroller=cc,color='FaceLeds:yellow'), transitions={'success': 'pre_ssl'})
        #
        #     smach.StateMachine.add(
        #         'pre_ssl', MoveHeadPepper(controller=hc,_hh='center', _hv='up', wait=2, speed=0.05),
        #         transitions={'success': 'Counter_look'})
        #
        #     smach.StateMachine.add(
        #         'Counter_look', Counter(numbers=15),
        #         transitions={'end': 'look_success', 'success': 'SSL'},
        #         remapping={'counter_input': 'iteration', 'counter_output': 'iteration'})
        #
        #     smach.StateMachine.add('SSL', Ssl(sensor=rs_ssl),
        #                            transitions={'success': 'LookSSL', 'no_sound': 'Counter_look'},
        #                            remapping={'input_angle_horizontal': 'horizontal_direction','output_angle_horizontal': 'horizontal_direction'})
        #
        #     smach.StateMachine.add(
        #         'LookSSL', MoveHeadPepper(controller=hc, _hv='up', wait=2, speed=0.25),
        #         transitions={'success': 'Counter_look'},
        #         remapping={'head_horizontal': 'horizontal_direction'})
        #
        # smach.StateMachine.add('Look_Statemaschine', sm_look, transitions={'look_success': 'Iterate'})

    # Introspection viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/ITELLIGENCE_1')
    sis.start()

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    main()
