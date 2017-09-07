import smach
import rospy
import math


class Ssl(smach.State):
    def __init__(self, sensor):
        self.sensor = sensor
        smach.State.__init__(self, outcomes=['success', 'no_sound'], input_keys=['input_angle_horizontal'], output_keys=['output_angle_horizontal'])

    def execute(self, userdata):
        angle = self.sensor.getData()
        if angle == None:
            rospy.sleep(2)
            return 'no_sound'
        else:
            degree = math.degrees(angle)
            userdata.output_angle_horizontal = userdata.input_angle_horizontal + degree
        return 'success'
