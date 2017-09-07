import rospy
from clf_perception_vision.msg import ExtendedPeople


class PersonSensor():
    def __init__(self):
        self.people = []
        self.people_sensor = rospy.Subscriber("/clf_perception_vision/people/raw/transform", ExtendedPeople,
                                              self.people_callback)
        print("Init Person Sensor")

    def people_callback(self, data):
        self.people = []
        for p in data.persons:
            self.people.append(p)

    def getDetectedPerson(self):
        self.returnval = self.people
        self.people = []
        return self.returnval
