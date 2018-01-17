#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from clf_perception_vision.msg import ExtendedPeople
from clf_perception_vision.srv import DoIKnowThatPerson, DoIKnowThatPersonRequest, DoIKnowThatPersonResponse
from threading import Lock
import rospy


"""

"""


class CheckForPersonState(EventState):
    """
    Checks whether or not the robot sees a Person

    -- name         String      Name of the Person to search for
    -- timeout      float       Timeout in seconds

    <= found                    The Person was found
    <= not_found                The Person was not found
    <= failed                   Execution failed
    """

    def __init__(self, name, timeout=2.0):
        """Constructor"""

        super(CheckForPersonState, self).__init__(outcomes=['found', 'not_found', 'failed'])

        self._name = name
        self._mutex = Lock()
        self._people = []
        self._timeout = timeout
        self._start_time = None
        self._callback_received = False
        self._transform_topic = '/clf_perception_vision/people/raw/transform'
        self._service_name = 'pepper_face_identification'

        self._service_proxy = ProxyServiceCaller({self._service_name: DoIKnowThatPerson})
        self._transform_listener = ProxySubscriberCached({self._transform_topic: ExtendedPeople})

    def people_callback(self, data):
        self._mutex.acquire()
        self._callback_received = True
        self._people = []
        for p in data.persons:
            self._people.append(p)
            self._transform_listener.unsubscribe_topic(self._transform_topic)
        self._mutex.release()

    def execute(self, userdata):
        """wait for transform callback, check all transforms for requested person"""
        elapsed = rospy.get_rostime() - self._start_time
        if elapsed > self._timeout:
            return 'not_found'
        self._mutex.acquire()
        if self._callback_received:
            known = False
            for p in self._people:
                request = DoIKnowThatPersonRequest()
                request.name = self.name
                request.transform_id = p.transformid
                response = self._service_proxy.call(self._service_name, request)
                if response.known:
                    known = True
            self._mutex.release()
            if known:
                return 'found'
            else:
                return 'not_found'
        self._mutex.release()

    def on_enter(self, userdata):
        """Subscribe to transforms, start timeout-timer"""
        self._start_time = rospy.get_rostime()
        self._transform_listener.subscribe(self._transform_topic, ExtendedPeople, self.people_callback)

    def on_exit(self, userdata):
        self._transform_listener.unsubscribe_topic()

    def on_stop(self):
        self._transform_listener.unsubscribe_topic()
