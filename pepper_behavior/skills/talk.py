import rospy
import smach


class Talk(smach.State):
    def __init__(self, controller, id=None, text=None, textblock='greeting'):
        self.id = id
        self.textblock = textblock
        self.text = text
        if  self.textblock == 'greeting':
            self.text = ['Herzlich Willkommen, mein Name ist Pepper. Mit mir wird zur Zeit am ceitaek der Universitaet Bielefeld geforscht.',
                     'Herzlich willkommen auf der Messe. Mein Name ist Pepper.',
                     'Hallo zusammen, ich bin Pepper! Ich freue mich bei der Messe dabei zu sein.',
                     'Hallo zusammen, ich bin Pepper! Ich unterstuetze derzeit die Forschung am ceitaek der Universitaet Bielefeld.',
                     'Herzlich Willkommen, mein Name ist Pepper ich bin ein Roboter vom ceitaek der Universitaet Bielefeld.']
        elif self.textblock == 'answer':
            self.text = [
                'Mir geht es sehr gut. Ich freue mich heute dabei zu sein.',
                'Ich wurde von Softbaenk Robotics entwickelt ich werde zurzeit am ceitaek der Universitaet Bielefeld weiter entwickelt. ',
                'Ich bin ein sozialer Roboter, der auf die Interaktion mit Menschen spezialisiert ist.',
                'Wir zeigen Ihnen auf der Messe, wie smarte, innovative Loesungen Ihr Unternehmen fuer '
                'die digitale Zukunft vorbereiten.',
		'Ich habe Sie leider nicht verstanden.',
		'Ich habe Sie leider nicht verstanden.',
		'Ich habe Sie leider nicht verstanden.',
		'Ich habe Sie leider nicht verstanden.']
        else:
            self.text = ['Es gibt dieses Jahr wieder eine Menge Demos. Weitere Infos sehen Sie dort.',
                     'Werfen Sie nach den Vortraegen doch einfach mal einen Blick in die Ausstellung. Schauen Sie doch dort.',
		     'Unterhalten Sie sich nach den Vortraegen doch mit mir in der Ausstellung. Weitere Infos gibt es hier.',
                     'Wir haben dieses Jahr wieder eine Menge Demos. Fuer weitere Informationen in diese Richtung.',
                     'Unterhalten Sie sich nach den Vortraegen doch mit mir in der Ausstellung. Weitere Demos sehen sie dort.']
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
