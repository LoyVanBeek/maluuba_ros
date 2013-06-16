#!/usr/bin/env python
'''Run ~/ros/fuerte/tue/user/sjoerd/google_speech_to_text/src recognizer.py, this works best somehow. Very experimental'''

import roslib
roslib.load_manifest('maluuba_ros')
import rospy

from std_msgs.msg import String
from maluuba_ros.srv import Interpret
from maluuba_ros.msg import Interpretation

rospy.init_node("listen_and_interpret")

service = rospy.ServiceProxy('maluuba/interpret', Interpret)
publisher = rospy.Publisher("maluuba/interpretations", Interpretation)


def interpret_string(msg):
    #import pdb; pdb.set_trace()
    rospy.loginfo("Heard '{0}'".format(msg.data))
    try:
        response = service(msg.data)
        interpretation = response.interpretation
        display = str(interpretation)
        lines = display.split("""\n""")
        display = "\n".join(
            line for line in lines if "''" not in line and '[]' not in line)
        
        rospy.loginfo(display)
        publisher.publish(interpretation)

    except Exception, e:
        rospy.logerr(e)

subscriber = rospy.Subscriber("/speech/output", String, interpret_string)

rospy.spin()
