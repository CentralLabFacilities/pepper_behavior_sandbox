import rospy


class RosSub():
    def __init__(self, dataType, scope):
        self.data = None
        self.dataType = dataType
        self.scope = scope
        self.people_sensor = rospy.Subscriber(name=self.scope, data_class=self.dataType, callback=self.callback)
        print("Init RosSub Sensor with Scope %s and Datatype %s" % (self.scope, self.dataType))

    def callback(self, data):
        self.data = None
        self.data = data.data

    def getData(self):
        self.returnval = self.data
        self.data = 0.0
        return self.returnval
