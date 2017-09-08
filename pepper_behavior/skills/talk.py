import rospy
import smach


class Talk(smach.State):
    def __init__(self, controller, id=None, text=None, textblock='greeting'):
        self.id = id
        self.textblock = textblock
        self.text = text
        if  self.textblock == 'greeting':
            self.text = ['Herzlich Willkommen, mein Name ist Pepper ich bin ein Roboter und mit mir wird zur Zeit am ceitaek der Universitaet Bielefeld geforscht.',
                     'Herzlich willkommen auf der \\readmode=word\\eitellijnz\\readmode=sent\\ woerld 2017! Mein Name ist Pepper.',
                     'Hallo, ich bin Pepper! Ich unterstuetze derzeit die Forschung am ceitaek der Universitaet Bielefeld und freue mich bei der \\readmode=word\\eitellijnz\\readmoed=sent\\ woerld 2017 dabei sein zu duerfen.',
                     'Herzlich Willkommen, mein Name ist Pepper ich bin ein Roboter und mit mir wird zur Zeit am ceitaek der Universitaet Bielefeld geforscht.']
        elif self.textblock == 'answer':
            self.text = [
                'Mir geht es sehr gut. Ich freue mich heute auf der \\readmode=word\\eitellijnz\\readmode=sent\\ woerld dabei sein zu duerfen.',
                'Ich wurde von Softbank Robotics entwickelt und bin darauf trainiert in unterschiedlichen '
                'Mensch-Maschine Szenarien zu unterstuetzen. Ich arbeite zurzeit am ceitaek der Universitaet ',
                'Ich bin ein humanoider Roboter, der auf die Interaktion mit Menschen spezialisiert ist. Ich '
                'erkenne Emotionen und kann auf die Emotionen von Menschen reagieren.',
                'Wir zeigen Ihnen auf der \\readmode=word\\eitellijnz\\readmode=sent\\ woerld, wie smarte, innovative Loesungen Ihr Unternehmen fuer '
                'die digitale Zukunft vorbereiten.']
        else:
            print('use second textblock')
            self.text = ['Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung. Weitere Infos sehen sie hier.',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung. Weitere Infos sehen sie hier',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung. Weitere Infos sehen sie hier',
                     'Wir haben dieses Jahr wieder eine Menge Demoszenarien zu Industrie 4.0 und Internet of Zhings '
                     'Werfen Sie nach der Key-Note doch einfach mal einen Blick in unsere Ausstellung. Weitere Infos sehen sie hier']
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
