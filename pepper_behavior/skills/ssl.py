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
            normed = userdata.input_angle_horizontal + degree
            if normed > 180:
                normed = normed -360
            elif normed < -180:
                normed = normed + 360
            userdata.output_angle_horizontal = normed
        return 'success'
