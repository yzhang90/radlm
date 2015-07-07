#!/usr/bin/env python

import roslib
roslib.load_manifest('cmu_monitors')
import rospy
from landshark_msgs.msg import BoolStamped

class ModeSwitcher():
	def __init__(self):
		rospy.init_node('change_obstacle_avoidance_state')

		self.lastMsg = None
		
		self.modePublisher = rospy.Publisher('/landshark/cmu_dynamic_window_monitor/control', BoolStamped)
		modeSubscriber = rospy.Subscriber('/landshark/cmu_dynamic_window_monitor/status', BoolStamped, self.onModeReceived)

	def onModeReceived(self, modeMsg):
		if self.lastMsg == None:
			self.lastMsg = modeMsg
		elif self.lastMsg.data != modeMsg.data:
			rospy.signal_shutdown('Done')
		else:
			msg = BoolStamped()
			msg.data = not modeMsg.data

			self.modePublisher.publish(msg)

if __name__ == "__main__":
	modeSwitcher = ModeSwitcher()
	rospy.spin()
	

