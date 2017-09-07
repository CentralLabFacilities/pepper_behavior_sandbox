import smach
import rospy
import math


class Ssl(smach.State):
    def __init__(self, sensor):
        self.sensor = sensor
        smach.State.__init__(self, outcomes=['success', 'no_sound'], output_keys=['angle_horizontal'])

    def execute(self, userdata):
        angle = self.sensor.getData()
        if angle == None:
            rospy.sleep(2)
            return 'no_sound'
        else:
            degree = math.degrees(angle)
            userdata.angle_horizontal = degree
        return 'success'
