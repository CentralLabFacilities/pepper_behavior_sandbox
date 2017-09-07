import rospy
import smach


class Talk(smach.State):
    def __init__(self, controller, id=None, text=None,textblock='greeting'):
        self.id = id
        self.textblock = textblock
        if  self.textblock:
            self.text = ['Hallo, ich bin Pepper! Herzlich willkommen auf der itelligence World 2017! Ich bin ein humanoider'
                     ' Roboter und arbeite zur Zeit am CITEC der Universitaet Bielefeld.'
                     'Hello one',
                     'Hello two',
                     'Hello three']
        else:
            self.text = ['Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung.',
                     'explain one',
                     'explain two',
                     'explain three']
        self.say = text
        if self.id:
            input_k = []
        else:
            input_k = ['id']
        smach.State.__init__(self, outcomes=['success'], input_keys=input_k)
        self.talk = controller

    def execute(self, userdata):
        if self.id:
            talk = self.text[self.id]
        elif self.say:
            talk = self.say
        else:
            talk = self.text[userdata.text]
        result = self.talk.say_something(talk)
        return result
