import rospy
import smach


class Talk(smach.State):
    def __init__(self, controller, id=None, text=None, textblock='greeting'):
        self.id = id
        self.textblock = textblock
        self.text = text
        if  self.textblock == 'greeting':
            self.text = ['Hallo, ich bin Pepper! Herzlich willkommen auf der itelligence World 2017! Ich bin ein',
                     ' Willkommen auf der itelligence World!.',
                     'Herzlich willkommen auf der itelligence World 2017! Mein Name ist Pepper.',
                     'Hallo, ich bin Pepper! Ich unterstütze derzeit die Forschung am CITEC der Universitae Bielefeld freue mich bei der itelligence World 2017 dabei sein zu dürfen.',
                     'Roboter und mit mir wird zur Zeit am CITEC der Universitaet Bielefeld geforscht.']
        else:
            print('use second textblock')
            self.text = ['Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung.',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung.',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung.',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung.']
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
            talk = self.text[userdata.id]
        result = self.talk.say_something(talk)
        return result
