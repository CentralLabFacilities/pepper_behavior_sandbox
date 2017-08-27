#!/usr/bin/env python

import time
import rospy
import smach
import roslib
import smach_ros
import actionlib
from std_msgs.msg import String
from math import radians, degrees
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from people_msgs.msg import Person, People
from naoqi_bridge_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackActionGoal, SpeechWithFeedbackGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class DataAcutators:
    def __init__(self):
        self.nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.speech_as = actionlib.SimpleActionClient('/naoqi_tts_feedback', SpeechWithFeedbackAction)
        rospy.loginfo("Connecting to /move_base...")
        self.nav_as.wait_for_server()
        rospy.loginfo("Connected.")

    def set_nav_goal(self, x, y, q0, q1, q2, q3):
        try:
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = '/map'  # Note: the frame_id must be map
            mb_goal.target_pose.pose.position.x = x
            mb_goal.target_pose.pose.position.y = y
            mb_goal.target_pose.pose.position.z = 0.0  # z must be 0.0 (no height in the map)
            # Orientation of the robot is expressed in the yaw value of euler angles
            mb_goal.target_pose.pose.orientation = Quaternion(q0, q1, q2, q3)
            self.nav_as.send_goal(mb_goal)
            rospy.loginfo("Waiting for result...")
            self.nav_as.wait_for_result()
            result = str(self.nav_as.get_state())
            # 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
        except Exception, e:
            return str(5)
        return result

    def say_something(self, _text):
        tts_goal = SpeechWithFeedbackGoal()
        tts_goal.say = _text
        self.speech_as.send_goal(tts_goal)
        rospy.loginfo("Waiting for result...")
        self.speech_as.wait_for_result()
        result = str(self.nav_as.get_state())
        # 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
        return result


class DataSensors:
    def __init__(self):
        self.speech_rec_context = rospy.Subscriber("/pepper_robot/speechrec/context", String, self.context_callback)
        self.people_sensor = rospy.Subscriber("/clf_detect_dlib_faces/people", People, self.people_callback)
        self.speech_rec_context = rospy.Subscriber("/clf_detect_objects_surb/objects", MarkerArray,
                                                   self.object_callback)
        self.current_context = ""
        self.current_objects = []
        self.current_people = []

    def reset_context(self):
        self.current_context = ""

    def reset_objects(self):
        self.current_objects = []

    def reset_people(self):
        self.current_people = []

    def context_callback(self, data):
        self.current_context = data.data.strip()

    def object_callback(self, data):
        self.reset_objects()
        for m in data.markers:
            self.current_objects.append(m.text)

    def people_callback(self, data):
        self.reset_people()
        for p in data.people:
            self.current_people.append(p.name)


class WaitForCommand(smach.State):
    def __init__(self, _ds):
        smach.State.__init__(self, outcomes=['table', 'init', 'gender', 'what', 'shelf', 'none'],
                             input_keys=['go_to_goal'], output_keys=['go_to_goal'])
        self.ds = _ds

    def execute(self, userdata):
        rospy.loginfo('Entering State WaitForCommand')
        while self.ds.current_context == "":
            time.sleep(0.1)
        if self.ds.current_context == "Drive to the table":
            rospy.loginfo('Drive to the table')
            self.ds.reset_context()
            userdata.go_to_goal = "table"
            return 'table'
        elif self.ds.current_context == "Go back to init position":
            rospy.loginfo('Go back to init position')
            self.ds.reset_context()
            userdata.go_to_goal = "init"
            return 'init'
        elif self.ds.current_context == "Drive to the shelf":
            rospy.loginfo('Drive to the shelf')
            self.ds.reset_context()
            userdata.go_to_goal = "shelf"
            return 'shelf'
        elif self.ds.current_context == "What is on the table":
            rospy.loginfo('What is on the table')
            self.ds.reset_context()
            return 'what'
        elif self.ds.current_context == "What is the gender of the person":
            rospy.loginfo('What is the gender of the person')
            self.ds.reset_context()
            return 'gender'
        rospy.logwarn('No valid command')
        self.ds.reset_context()
        return 'none'


class GoTo(smach.State):
    def __init__(self, _da):
        smach.State.__init__(self, outcomes=['arrived', 'fail'], input_keys=['go_to_goal'])
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State GoToTable')
        result = 0
        if userdata.go_to_goal == "init":
            self.da.say_something("Okay my friend, I will drive to the %s position" % userdata.go_to_goal)
            result = self.da.set_nav_goal(0.99, 0.78, 0.0, 0.0, -0.34, 0.93)
        elif userdata.go_to_goal == "shelf":
            self.da.say_something("Going to the %s" % userdata.go_to_goal)
            result = self.da.set_nav_goal(3.24, -2.46, 0.0, 0.0, -0.55, 0.83)
        elif userdata.go_to_goal == "table":
            self.da.say_something("With pleasure. Trying to reach the %s" % userdata.go_to_goal)
            result = self.da.set_nav_goal(1.05, -1.99, 0.0, 0.0, 0.98, 0.16)
        elif userdata.go_to_goal == "":
            self.da.say_something("That is not an actual location")
            rospy.logwarn('Location is empty')
            return 'fail'
        if result == "3":
            self.da.say_something("I am done. What shall I do?")
            return 'arrived'
        else:
            self.da.say_something("I could not reach the table, oh no!")
            rospy.logwarn('Could not reach table: %s' % str(result))
            return 'fail'


class WhatIs(smach.State):
    def __init__(self, _ds, _da):
        smach.State.__init__(self, outcomes=['report'])
        self.ds = _ds
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State WhatIs')
        see = "I see "
        objects = self.ds.current_objects
        if len(objects) < 1:
            self.da.say_something("Unfortunately, I can not see any objects")
        else:
            for o in objects:
                self.da.say_something(see + str(o))
        self.ds.reset_objects()
        return 'report'


class WhoIs(smach.State):
    def __init__(self, _ds, _da):
        smach.State.__init__(self, outcomes=['whois'])
        self.ds = _ds
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State WhatIs')
        people = self.ds.current_people
        if len(people) < 1:
            self.da.say_something("I can not see any person, I am sorry.")
        else:
            many = "I see " + str(len(people))
            self.da.say_something(many)
            for o in people:
                one = "One person is "
                age = "The age is "
                self.da.say_something(one + str(o).split(':')[0])
                self.da.say_something(age + str(o).split(':')[1])
        self.ds.reset_people()
        return 'whois'


def main():
    rospy.init_node('geniale_pepper_state_machine')
    ds = DataSensors()
    da = DataAcutators()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.go_to_goal = ""

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT', WaitForCommand(ds),
                               transitions={'table': 'GOTO',
                                            'init': 'GOTO',
                                            'shelf': 'GOTO',
                                            'what': 'WHATIS',
                                            'gender': 'WHOIS',
                                            'none': 'WAIT'})

        smach.StateMachine.add('GOTO', GoTo(da),
                               transitions={'arrived': 'WAIT',
                                            'fail': 'WAIT'})

        smach.StateMachine.add('WHATIS', WhatIs(ds, da),
                               transitions={'report': 'WAIT'})

        smach.StateMachine.add('WHOIS', WhoIs(ds, da),
                               transitions={'whois': 'WAIT'})

    # Introspection viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/GENIALE')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
