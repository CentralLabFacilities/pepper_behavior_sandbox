#!/usr/bin/env python

import qi
import sys
from optparse import OptionParser
from naoqi_driver.naoqi_node import NaoqiNode

class TTS(object):
    def __init__(self):
        super(TTS, self).__init__()
        app.start()
        session = app.session
        self.memory = session.service("ALMemory")
        self.tts = session.service("ALTextToSpeech")
        self.motion = session.service("ALMotion")
        self.animation = session.service("ALAnimationPlayer")
        self.last_said = ""
        self.pitch = 1.0
        self.tts.setParameter("pitchShift", self.pitch)

    # (re-) connect to NaoQI:

    def run(self):
        self.tts.say("TEST")



if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--pip", dest="pip", default=True)
    parser.add_option("--pport", dest="pport", default=True)
    (options, args) = parser.parse_args()
    try:
        connection_url = options.pip + ":" + options.pport
        app = qi.Application(["PepperVideoIntroduction", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi")
        sys.exit(1)

    tts = TTS(app)
    tts.run()

