import rospy
import smach


class Talk(smach.State):
    def __init__(self, controller, id=None, text=None, textblock='greeting'):
        self.id = id
        self.textblock = textblock
        self.text = text
        if  self.textblock == 'greeting':
            self.text = ['Herzlich Willkommen, mein Name ist Pepper ich bin ein sozialer Roboter und mit mir wird zur Zeit am ceitaek der Universitaet Bielefeld geforscht.',
                     'Herzlich willkommen auf der Messe. Mein Name ist Pepper.',
                     'Hallo, ich bin Pepper! Ich unterstuetze derzeit die Forschung am ceitaek der Universitaet Bielefeld und freue mich bei der Messe dabei zu sein.',
                     'Herzlich Willkommen, mein Name ist Pepper ich bin ein sozialer Roboter und mit mir wird zur Zeit am ceitaek der Universitaet Bielefeld geforscht.']
        elif self.textblock == 'answer':
            self.text = [
                'Mir geht es sehr gut. Ich freue mich heute auf der Messe dabei sein zu duerfen.',
                'Ich wurde von Softbaenk Robotics entwickelt und bin darauf trainiert in unterschiedlichen.'
                'Mensch-Maschine Szenarien zu unterstuetzen. Ich arbeite zurzeit am ceitaek der Universitaet Bielefeld. ',
                'Ich bin ein sozialer Roboter, der auf die Interaktion mit Menschen speciahlisiert ist.',
                'Wir zeigen Ihnen auf der Messe, wie smarte, innovative Loesungen Ihr Unternehmen fuer '
                'die digitale Zukunft vorbereiten.']
        else:
            self.text = ['Wir haben dieses Jahr wieder eine Menge Demoszenarien.'
                     'Werfen Sie nach den Vortraegen doch einfach mal einen Blick in unsere Ausstellung. Weitere Infos sehen sie hier.',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien. '
                     'Werfen Sie nach den Vortraegen doch einfach mal einen Blick in unsere Ausstellung. Weitere Infos sehen sie hier',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien. '
                     'Werfen Sie nach den Vortraegen doch einfach mal einen Blick in unsere Ausstellung. Weitere Infos sehen sie hier',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien. '
                     'Werfen Sie nach den Vortraegen doch einfach mal einen Blick in unsere Ausstellung. Weitere Infos sehen sie hier']
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
