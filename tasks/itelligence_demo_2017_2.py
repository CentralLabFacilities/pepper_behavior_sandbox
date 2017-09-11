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
from pepper_behavior.skills.SpeechAnalyser import SpeechAnalyser
from pepper_behavior.skills.state_publisher import StatePublisher

from pepper_behavior.actuators.head_control import HeadControlPepper
from pepper_behavior.actuators.talk import TalkControllerPepper
from pepper_behavior.actuators.ros_string_pub import RosStringPub
from pepper_behavior.actuators.arm_control import LeftArmControlPepper

from pepper_behavior.sensors.person_sensor import PersonSensor
from pepper_behavior.sensors.ros_sub import RosSub
from pepper_behavior.sensors.speechsensor import SpeechSensor
from pepper_behavior.sensors.id_sensor import IdSensor



def main():
    simulation = False
    rospy.init_node('intelligence_pepper_state_machine')
    speechsensor = SpeechSensor()
    hc = HeadControlPepper()
    tc = TalkControllerPepper(sim=simulation)
    ps = PersonSensor()
    animation_pub = RosStringPub('/pepper_robot/animation_player')
    st = RosStringPub('/pepper_robot/smach/state')
    cc = RosStringPub('/pepper_robot/leds')
    id = IdSensor()


    rospy.sleep(1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.questions = 0
    sm.userdata.vertical_angle = 0.0
    sm.userdata.horizontal_angle = 0.0
    sm.userdata.answer_id = 0
    sm.userdata.answer_counter = 0
    sm.userdata.useless = 0

    # Open the container
    with sm:
        smach.StateMachine.add(
            'Init_Talk', Talk(controller=tc, text='Demo startet.'),
            transitions={'success': 'Init_state'})

        smach.StateMachine.add('Init_state', StatePublisher(st, 'init', colorcontroller=cc, color='FaceLeds:cyan'),
                               transitions={'success': 'MoveHead_init'})

        smach.StateMachine.add(
            'MoveHead_init',
            MoveHeadPepper(_hv='up', _hh='center', controller=hc, wait=4, speed=0.1),
            transitions={'success': 'CalculatePersonPosition'})

        smach.StateMachine.add(
            'CalculatePersonPosition', CalculatePersonPosition(controller=ps, max_distance=1.5, sensor=id),
            transitions={'success': 'LookToPerson', 'repeat': 'CalculatePersonPosition',
                         'no_person_found': 'MoveHead_init', 'known': 'MoveHead_init'},
            remapping={'person_angle_vertical': 'vertical_angle', 'person_angle_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'LookToPerson', MoveHeadPepper(controller=hc, wait=1),
            transitions={'success': 'Animation'},
            remapping={'head_vertical': 'vertical_angle', 'head_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'Animation',
            AnimationPlayerPepper(controller=animation_pub, id=1, animationblock='greetings'),
            transitions={'success': 'Welcome_Talk'})

        smach.StateMachine.add(
            'Welcome_Talk', Talk(controller=tc, text='Hallo, ich bin Pepper ! Herzlich willkommen auf der Messe.'),
            transitions={'success': 'listen_state'})

        smach.StateMachine.add('listen_state', StatePublisher(st, 'listen_mode', colorcontroller=cc, color='FaceLeds:cyan'),
                               transitions={'success': 'CalculatePersonPosition_save'})

        smach.StateMachine.add(
            'CalculatePersonPosition_save',
            CalculatePersonPosition(controller=ps, max_distance=1.5, onlyhorizontal=True, knownperson=False),
            transitions={'success': 'LookToPerson_save', 'repeat': 'LookToPerson_save',
                         'no_person_found': 'LookToPerson_save', 'known': 'LookToPerson_save'},
            remapping={'old_vertical': 'vertical_angle', 'person_angle_vertical': 'vertical_angle',
                       'person_angle_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'LookToPerson_save', MoveHeadPepper(controller=hc, wait=0),
            transitions={'success': 'Counter_answers'},
            remapping={'head_vertical': 'vertical_angle', 'head_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'Counter_answers', Counter(numbers=3),
            transitions={'success': 'listen', 'end': 'question_state'},
            remapping={'counter_input': 'answer_counter', 'counter_output': 'answer_counter'})

        smach.StateMachine.add('listen', SpeechAnalyser(controller=speechsensor, wait=10),
                               transitions={'success': 'answer_state', 'reset':'search_state', 'no_cmd': 'CalculatePersonPosition_saveback'},
                               remapping={'msg_output': 'answer_id'})

        smach.StateMachine.add(
            'CalculatePersonPosition_saveback',
            CalculatePersonPosition(controller=ps, max_distance=1.5, onlyhorizontal=True, knownperson=False),
            transitions={'success': 'LookToPerson_saveback', 'repeat': 'CalculatePersonPosition_saveback',
                         'no_person_found': 'search_state', 'known': 'LookToPerson_saveback'},
            remapping={'old_vertical': 'vertical_angle', 'person_angle_vertical': 'vertical_angle',
                       'person_angle_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'LookToPerson_saveback', MoveHeadPepper(controller=hc, wait=0),
            transitions={'success': 'listen'},
            remapping={'head_vertical': 'vertical_angle', 'head_horizontal': 'horizontal_angle'})

        smach.StateMachine.add('search_state', StatePublisher(st, 'search_mode', colorcontroller=cc, color='FaceLeds:cyan'),
                               transitions={'success': 'Counter_answers_reset'})

        smach.StateMachine.add(
            'Counter_answers_reset', Counter(numbers=3,reset=True),
            transitions={'success': 'MoveHead_init', 'end': 'MoveHead_init'},
            remapping={'counter_input': 'answer_counter', 'counter_output': 'answer_counter'})


        smach.StateMachine.add('answer_state', StatePublisher(st, 'answer_mode', colorcontroller=cc, color='FaceLeds:cyan'),
                               transitions={'success': 'CalculatePersonPosition_answer'})

        smach.StateMachine.add(
            'CalculatePersonPosition_answer',
            CalculatePersonPosition(controller=ps, max_distance=1.5, onlyhorizontal=True, knownperson=False),
            transitions={'success': 'LookToPerson_answer', 'repeat': 'LookToPerson_answer',
                         'no_person_found': 'LookToPerson_answer', 'known': 'LookToPerson_answer'},
            remapping={'old_vertical': 'vertical_angle', 'person_angle_vertical': 'vertical_angle',
                       'person_angle_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'LookToPerson_answer', MoveHeadPepper(controller=hc, wait=0),
            transitions={'success': 'Animation_talking'},
            remapping={'head_vertical': 'vertical_angle', 'head_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'Animation_talking',
            AnimationPlayerPepper(controller=animation_pub, animationblock='talking'),
            transitions={'success': 'Answer_Question'}, remapping={'id': 'answer_id'})

        smach.StateMachine.add(
            'Answer_Question',
            Talk(controller=tc, textblock='answer'), transitions={'success': 'listen_state'},
            remapping={'id': 'answer_id'})

        smach.StateMachine.add('question_state', StatePublisher(st, 'question_mode', colorcontroller=cc, color='FaceLeds:cyan'),
                               transitions={'success': 'Animation_answer'})

        smach.StateMachine.add(
            'Animation_answer',
            AnimationPlayerPepper(controller=animation_pub, animation='animations/Stand/Gestures/ShowTablet_2'),
            transitions={'success': 'Question_Talk'})


        smach.StateMachine.add(
            'Question_Talk', Talk(controller=tc, text='Wie gefaellt Ihnen die Messe? Antworten Sie bitte mit einem der angezeigten Saetze.'),
            transitions={'success': 'listen_question'})

        smach.StateMachine.add('listen_question', SpeechAnalyser(controller=speechsensor, wait=10),
                               transitions={'success': 'Animation_answer_end', 'reset':'search_state', 'no_cmd': 'CalculatePersonPosition_saveback'},
                               remapping={'msg_output': 'useless'})

        smach.StateMachine.add(
            'Animation_answer_end',
            AnimationPlayerPepper(controller=animation_pub, animation='animations/Stand/Gestures/Thinking_4'),
            transitions={'success': 'Question_Talk_answer'})

        smach.StateMachine.add(
            'Question_Talk_answer', Talk(controller=tc, text='Vielen Dank fuer ihre Antwort. Ich beantworte gerne weitere Fragen.'),
            transitions={'success': 'Counter_answers_reset_2'})


        smach.StateMachine.add(
            'Counter_answers_reset_2', Counter(numbers=3, reset=True),
            transitions={'success': 'listen_state', 'end': 'listen_state'},
            remapping={'counter_input': 'answer_counter', 'counter_output': 'answer_counter'})

    # Introspection viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/ITELLIGENCE_2')
    sis.start()

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    main()
