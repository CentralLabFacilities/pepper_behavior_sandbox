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

from pepper_behavior.actuators.head_control import HeadControlPepper
from pepper_behavior.actuators.talk import TalkControllerPepper
from pepper_behavior.actuators.ros_string_pub import RosStringPub
from pepper_behavior.actuators.arm_control import LeftArmControlPepper

from pepper_behavior.sensors.person_sensor import PersonSensor
from pepper_behavior.sensors.ros_sub import RosSub
from pepper_behavior.sensors.speechsensor import SpeechSensor


def main():
    simulation = False
    rospy.init_node('intelligence_pepper_state_machine')
    speechsensor = SpeechSensor()
    hc = HeadControlPepper()
    tc = TalkControllerPepper(sim=simulation)
    ps = PersonSensor()
    animation_pub = RosStringPub('/pepper/animation_player')

    rospy.sleep(1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.questions = 0
    sm.userdata.vertical_angle = 0.0
    sm.userdata.horizontal_angle = 0.0
    sm.userdata.answer_id = 0.0


    # Open the container
    with sm:
        smach.StateMachine.add(
            'Init_Talk', Talk(controller=tc, text='Demo startet.'),
            transitions={'success': 'MoveHead_init'})

        smach.StateMachine.add(
            'MoveHead_init',
            MoveHeadPepper(_hv='up', _hh='center', controller=hc, wait=5, speed=0.1),
            transitions={'success': 'CalculatePersonPosition'})

        smach.StateMachine.add(
            'CalculatePersonPosition', CalculatePersonPosition(controller=ps, max_distance=2.5),
            transitions={'success': 'LookToPerson', 'repeat': 'CalculatePersonPosition',
                         'no_person_found': 'MoveHead_init'},
            remapping={'person_angle_vertical': 'vertical_angle', 'person_angle_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'LookToPerson', MoveHeadPepper(controller=hc, wait=1),
            transitions={'success': 'Animation'},
            remapping={'head_vertical': 'vertical_angle', 'head_horizontal': 'horizontal_angle'})

        smach.StateMachine.add(
            'Animation',
            AnimationPlayerPepper(controller=animation_pub, id=0),
            transitions={'success': 'Welcome_Talk'})

        smach.StateMachine.add(
            'Welcome_Talk', Talk(controller=tc, text='Hallo, ich bin Pepper ! Herzlich willkommen auf der itelligence World '
                                                  '2017!'), transitions={'success': 'listen'})

        smach.StateMachine.add('listen', SpeechAnalyser(controller=SpeechSensor,wait=10),
                               transitions={'success':'Animation_talking', 'no_cmd':'CalculatePersonPosition_saveback'},
                               remapping={'msg_output': 'answer_id'})

        smach.StateMachine.add(
            'CalculatePersonPosition_saveback', CalculatePersonPosition(controller=ps, max_distance=1.0),
            transitions={'success': 'listen', 'repeat': 'CalculatePersonPosition_saveback',
                         'no_person_found': 'MoveHead_init'},
            remapping={'person_angle_vertical': 'vertical_angle', 'person_angle_horizontal': 'horizontal_angle'})


        smach.StateMachine.add(
            'Animation_talking',
            AnimationPlayerPepper(controller=animation_pub, animationblock='talking'),
            transitions={'success': 'Answer_Question'}, remapping={'id': 'answer_id'})

        smach.StateMachine.add(
            'Answer_Question',
            Talk(controller=tc,textblock='answer'), transitions={'success': 'listen'},
            remapping={'id': 'answer_id'})


    # Introspection viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/ITELLIGENCE')
    sis.start()

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    main()
