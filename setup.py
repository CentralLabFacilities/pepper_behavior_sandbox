import subprocess

from setuptools import setup, find_packages

version = "master"

filename = "master"

setup(name="pepper_behavior_sandbox",

      version=filename,

      description="",

      long_description="",

      author="Florian Lier and Nils Neumann",

      author_email="flier[at]techfak.uni-bielefeld.de and nneumann[at]techfak.uni-bielefeld.de",

      url="",

      download_url="",

      packages=find_packages(exclude=["*.tests",

                                      "*.tests.*",

                                      "tests.*",

                                      "tests"]),

      scripts=["tasks/itelligence_demo_2017_1.py",
               "tasks/itelligence_demo_2017_2.py",
               "tasks/itelligence_reject_speech.py",
               "smach/pepper_smach_geniale_2017.py"
               ],

      keywords=['Behavior'],

      license="LGPLv3",

      classifiers=['Development Status :: Beta',

                   'Environment :: Console',

                   'Environment :: Robotics',

                   'Intended Audience :: Developers',

                   'License :: OSI Approved :: GNU Library or ' +

                   'Lesser General Public License (LGPL)',

                   'Operating System :: OS Independent',

                   'Programming Language :: Python',

                   'Topic :: Text Processing :: Markup :: XML'],

      zip_safe=False

      )

# Make scripts executable

subprocess.call(["chmod -R ugo+x tasks"], shell=True)
