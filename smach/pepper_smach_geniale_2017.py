#!/usr/bin/env python

import tf
import time
import copy
import math
import rospy
import smach
import random
import smach_ros
import actionlib
from threading import Lock
from std_msgs.msg import String, Bool
from clf_perception_vision.srv import *
from people_msgs.msg import Person, People
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from clf_perception_vision.msg import ExtendedPeople, ExtendedPersonStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from naoqi_bridge_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackActionGoal, SpeechWithFeedbackGoal


class DataAcutators:
    def __init__(self):
        self.down = '25.0:0.0'
        self.drive = '15.0:0.0'
        self.normal = '0.0:0.0'
        self.people = '-15.0:0.0'
        self.current_goal = MoveBaseGoal()
        self.nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.speech_as = actionlib.SimpleActionClient('/naoqi_tts_feedback', SpeechWithFeedbackAction)
        self.head_pub = rospy.Publisher('/pepper_robot/head/pose', String, queue_size=1)
        self.obj_compute = rospy.Publisher('/clf_detect_objects_surb/compute', Bool, queue_size=1)
        self.people_compute = rospy.Publisher('/clf_detect_dlib_faces/compute', Bool, queue_size=1)
        self.dd = rospy.Publisher('/pepper_robot/drivedirect', String, queue_size=1)
        self.td = rospy.Publisher('/pepper_robot/turndirect', String, queue_size=1)
        rospy.loginfo("Connecting to /move_base...")
        self.nav_as.wait_for_server()
        rospy.loginfo("Connected.")

    def compute_people(self, _data):
        b = Bool()
        b.data = _data
        self.people_compute.publish(b)
        rospy.loginfo("Detecting People: %s" % str(b.data))

    def compute_objects(self, _data):
        b = Bool()
        b.data = _data
        self.obj_compute.publish(b)
        rospy.loginfo("Detecting Objects: %s" % str(b.data))

    def set_head_down(self):
        target = String()
        target.data = self.down
        self.head_pub.publish(target)

    def set_head_drive(self):
        target = String()
        target.data = self.drive
        self.head_pub.publish(target)

    def set_head_normal(self):
        target = String()
        target.data = self.normal
        self.head_pub.publish(target)

    def set_head_people(self):
        target = String()
        target.data = self.people
        self.head_pub.publish(target)

    def look_closer(self):
        try:
            self.set_head_drive()
            self.dd.publish("0.15:0.0:0.0")
        except Exception, e:
            return str(5)
            self.set_head_normal()
        self.set_head_normal()
        return "3"

    def turn(self, _value):
        try:
            self.set_head_drive()
            self.td.publish(_value)
        except Exception, e:
            return str(5)
            self.set_head_normal()
        self.set_head_normal()
        return "3"

    def set_nav_goal(self, x, y, q0, q1, q2, q3):
        try:
            self.set_head_drive()
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = '/map'  # Note: the frame_id must be map
            mb_goal.target_pose.pose.position.x = x
            mb_goal.target_pose.pose.position.y = y
            mb_goal.target_pose.pose.position.z = 0.0  # z must be 0.0 (no height in the map)
            # Orientation of the robot is expressed in the yaw value of euler angles
            mb_goal.target_pose.pose.orientation = Quaternion(q0, q1, q2, q3)
            self.current_goal = mb_goal
            self.nav_as.send_goal(mb_goal)
            rospy.loginfo("Waiting for result...")
            self.nav_as.wait_for_result()
            result = str(self.nav_as.get_state())
            # 3 is SUCCESS, 4 is ABORTED (couldnt get there), 5 REJECTED (the goal is not attainable)
        except Exception, e:
            return str(5)
            self.set_head_normal()
        self.set_head_normal()
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


class ClosestPersonSensor:
    def __init__(self):
        self.mutex = Lock()
        self.people = []
        self.people_sensor = rospy.Subscriber("/clf_perception_vision/people/raw/transform", ExtendedPeople,
                                              self.people_callback)
        print("Init Person Sensor")

    def people_callback(self, data):
        self.mutex.acquire()
        self.people = []
        for p in data.persons:
            self.people.append(p)
        self.mutex.release()

    def distance(self, trans):
        dist = math.sqrt(trans.x * trans.x + trans.y * trans.y + trans.z * trans.z)
        return dist

    def get_closest(self):
        self.mutex.acquire()
        tmp_people = copy.deepcopy(self.people)
        self.mutex.release()
        min_dist = 10000.0
        transform_id = None
        for p in tmp_people:
            pose = p.pose
            dist = self.distance(pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                rospy.loginfo("New min distance %f" % min_dist)
                transform_id = p.transformid
        return transform_id


class IdSensor:
    def __init__(self):
        rospy.loginfo("Waiting for Service Person ID")
        rospy.wait_for_service('pepper_face_identification')
        rospy.loginfo("Init Person ID Sensor")

    def identify(self, _data):
        try:
            person_id = rospy.ServiceProxy('pepper_face_identification', DoIKnowThatPerson)
            resp = person_id(_data)
            return resp.known, resp.name
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)


class DataSensors:
    def __init__(self):
        self.id_sensor = IdSensor()
        self.closest_person = ClosestPersonSensor()
        self.speech_rec_context = rospy.Subscriber("/pepper_robot/speechrec/context", String, self.context_callback)
        self.people_sensor = rospy.Subscriber("/clf_perception_vision/people/raw", ExtendedPeople, self.people_callback)
        self.objects_sensor = rospy.Subscriber("/clf_perception_surb/objects", MarkerArray, self.object_callback)
        self.current_context = ""
        self.current_objects = []
        self.current_people = []

    def identify(self):
        person_id = self.closest_person.get_closest()
        if person_id is not None:
            known, name = self.id_sensor.identify(person_id)
            return known
        else:
            rospy.logwarn("No person id found")
            return person_id

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
        for p in data.persons:
            self.current_people.append(p.name)


class WaitForCommand(smach.State):
    def __init__(self, _ds, _da):
        smach.State.__init__(self, outcomes=['table',
                                             'init',
                                             'persons',
                                             'closer',
                                             'what',
                                             'shelf',
                                             'sliding door',
                                             'exit door',
                                             'turn',
                                             'know',
                                             'none'],
                             input_keys=['go_to_goal'],
                             output_keys=['go_to_goal'])
        self.ds = _ds
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State WaitForCommand')
        self.da.set_head_normal()
        while self.ds.current_context == "":
            time.sleep(0.1)
        if self.ds.current_context == "Drive to the table":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.go_to_goal = "table"
            return 'table'
        elif self.ds.current_context == "Go back to init position":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.go_to_goal = "init"
            return 'init'
        elif self.ds.current_context == "Drive to the shelf":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.go_to_goal = "shelf"
            return 'shelf'
        elif self.ds.current_context == "Drive to sliding door":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.go_to_goal = "sliding door"
            return 'sliding door'
        elif self.ds.current_context == "Drive to exit door":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            userdata.go_to_goal = "exit door"
            return 'exit door'
        elif self.ds.current_context == "What objects do you see":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            return 'what'
        elif self.ds.current_context == "What is this":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            return 'what'
        elif self.ds.current_context == "Please look a little closer":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            return 'closer'
        elif self.ds.current_context == "Detect people please":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            return 'persons'
        elif self.ds.current_context == "Count the people":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            return 'persons'
        elif self.ds.current_context == "Tobi turn around":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            return 'turn'
        elif self.ds.current_context == "Do you know me":
            rospy.loginfo(self.ds.current_context)
            self.ds.reset_context()
            return 'know'
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
            self.da.say_something("Okay my friend, I will drive to the %s position." % userdata.go_to_goal)
            result = self.da.set_nav_goal(0.99, 0.78, 0.0, 0.0, -0.34, 0.93)
        elif userdata.go_to_goal == "shelf":
            self.da.say_something("Going to the %s." % userdata.go_to_goal)
            result = self.da.set_nav_goal(2.98, -2.35, 0.0, 0.0, -0.44, 0.89)
        elif userdata.go_to_goal == "table":
            self.da.say_something("With pleasure. Trying to reach the %s." % userdata.go_to_goal)
            result = self.da.set_nav_goal(1.05, -1.99, 0.0, 0.0, 0.98, 0.16)
        elif userdata.go_to_goal == "sliding door":
            self.da.say_something("That is not too far. Going to reach the %s." % userdata.go_to_goal)
            result = self.da.set_nav_goal(3.14, -1.01, 0.0, 0.0, -0.17, 0.98)
        elif userdata.go_to_goal == "exit door":
            self.da.say_something("%s it is." % userdata.go_to_goal)
            result = self.da.set_nav_goal(3.44, 0.28, 0.0, 0.0, 0.65, 0.75)
        elif userdata.go_to_goal == "":
            self.da.say_something("That is not an actual location")
            rospy.logwarn('Location is empty')
            return 'fail'
        if result == "3":
            self.da.say_something("I am done. Can I do something else?")
            return 'arrived'
        else:
            self.da.say_something("I could not reach the %s, oh no!" % userdata.go_to_goal)
            rospy.logwarn('Could not reach table: %s' % str(result))
            return 'fail'


class LookCloser(smach.State):
    def __init__(self, _da):
        smach.State.__init__(self, outcomes=['arrived', 'fail'], input_keys=['go_to_goal'])
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State LookCloser')
        result = self.da.look_closer()
        if result == "3":
            self.da.say_something("That is a little closer now. Is it?")
            return 'arrived'
        else:
            self.da.say_something("Could not look closer!")
            rospy.logwarn('Could not look closer %s' % str(result))
            return 'fail'


class WhatIs(smach.State):
    def __init__(self, _ds, _da):
        smach.State.__init__(self, outcomes=['report'])
        self.ds = _ds
        self.da = _da

    def execute(self, userdata):
        self.da.set_head_down()
        self.da.compute_objects(True)
        time.sleep(1.5)
        rospy.loginfo('Entering State WhatIs')
        see = "I see "
        objects = self.ds.current_objects
        if len(objects) < 1:
            self.da.say_something("Unfortunately, I can not see any objects")
        else:
            for o in objects:
                self.da.say_something(see + str(o))
        self.ds.reset_objects()
        self.da.compute_objects(False)
        return 'report'


class WhoIs(smach.State):
    def __init__(self, _ds, _da):
        smach.State.__init__(self, outcomes=['whois'])
        self.ds = _ds
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State WhatIs')
        self.da.compute_people(True)
        time.sleep(1.5)
        people = self.ds.current_people
        if len(people) < 1:
            self.da.say_something("I can not see any person, I am sorry.")
        else:
            many = "I see " + str(len(people))
            self.da.say_something(many)
            # for o in people:
            #    one = "One person is "
            #    age = "The age is "
            #    self.da.say_something(one + str(o).split(':')[0])
            #    self.da.say_something(age + str(o).split(':')[1])
        self.ds.reset_people()
        self.da.compute_people(False)
        return 'whois'


class Turn(smach.State):
    def __init__(self, _da):
        smach.State.__init__(self, outcomes=['turn'])
        self.da = _da

    def execute(self, userdata):
        rospy.loginfo('Entering State Turn')
        self.da.turn("-140.0")
        time.sleep(1.5)
        return 'turn'


class Know(smach.State):
    def __init__(self, _ds, _da):
        smach.State.__init__(self, outcomes=['result'])
        self.da = _da
        self.ds = _ds
        self.helper_text_know = ["Yes, I know you!", "I remember you!", "Yes, I know who you are!", "Well, yes I know you."]
        self.helper_text_dontknow = ["No, I don't know you!", "Sorry, I don't know who you are.", "Nope, I don't know you.", "Sorry, I cannot remember you."]

    def execute(self, userdata):
        rospy.loginfo('Entering State Know')
        self.da.set_head_people()
        result = self.ds.identify()
        rospy.loginfo("Person ID result %s" % str(result))
        if result is not None and result is not False:
            self.da.say_something(random.choice(self.helper_text_know))
        else:
            self.da.say_something(random.choice(self.helper_text_dontknow))
        return 'result'


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
        smach.StateMachine.add('WAIT', WaitForCommand(ds, da),
                               transitions={'table': 'GOTO',
                                            'init': 'GOTO',
                                            'shelf': 'GOTO',
                                            'sliding door': 'GOTO',
                                            'exit door': 'GOTO',
                                            'what': 'WHATIS',
                                            'closer': 'LOOKCLOSER',
                                            'persons': 'WHOIS',
                                            'turn': 'TURN',
                                            'know': 'KNOW',
                                            'none': 'WAIT'})

        smach.StateMachine.add('GOTO', GoTo(da),
                               transitions={'arrived': 'WAIT',
                                            'fail': 'WAIT'})

        smach.StateMachine.add('WHATIS', WhatIs(ds, da),
                               transitions={'report': 'WAIT'})

        smach.StateMachine.add('WHOIS', WhoIs(ds, da),
                               transitions={'whois': 'WAIT'})

        smach.StateMachine.add('LOOKCLOSER', LookCloser(da),
                               transitions={'arrived': 'WAIT',
                                            'fail': 'WAIT'})

        smach.StateMachine.add('TURN', Turn(da), transitions={'turn': 'WAIT'})

        smach.StateMachine.add('KNOW', Know(ds, da), transitions={'result': 'WAIT'})

    # Introspection viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/GENIALE')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
