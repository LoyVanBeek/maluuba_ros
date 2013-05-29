#!/usr/bin/env python
import roslib; roslib.load_manifest('maluuba_ros')
import rospy

from std_msgs.msg import String
from maluuba_ros.srv import Interpret

rospy.init_node("listen_and_interpret")

service = rospy.ServiceProxy('maluuba/interpret', Interpret)

def interpret_string(msg):
	try:
		response = service(msg.data)
		display = str(response)
		lines = display.split("""\n""")
		display = "\n".join(line for line in lines if "''" not in line and '[]' not in line)
		print display
	except Exception, e:
		rospy.logerr(e)

subcriber = rospy.Subscriber("/speech/output", String, interpret_string)

rospy.spin()