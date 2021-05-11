#!/usr/bin/env python

# Written by Simon Krogedal
# Script to translate the desired joint angles published for the serial model of the arm and publish the corresponding angles for acutated joints
# Subs to /joint_states and pubs in /phys_joint_states

import rospy
import math
from sensor_msgs.msg import JointState

class lift_arm_virt2phys(object):

	# Constructor
	def __init__(self):
		
		# Inititialise node
		rospy.init_node('virtual2physical_joint_publisher', anonymous=True)
	
		# Declare publisher to give joint values to arduino
		self._pub = rospy.Publisher('phys_joint_states', JointState, queue_size=10)
		
		# Variable to hold joint angles
		self._joints = JointState()
		self._joints.name =  ['main', 'crank', 'rack']
		self._joints.position = [0, 0, 0]
		
		# Subscribe to /joint_states
		rospy.Subscriber("joint_states", JointState, self._callback)
		
		# Various
		self.ctrl_c = False
		self.rate = rospy.Rate(10)
		rospy.on_shutdown(self._shutdownhook)
		
	# shutdown hook, works better than rospy.is_shutdown() (according to the Construct)
	def _shutdownhook(self):
		self.ctrl_c = True

	def calcBeta(self, theta1, theta3):
		# set up link lengths
		a2 = 180
		b1 = 90
		b2 = 160
		b3 = 70
		
		# diagonal lenght
		k_sqrd = a2*a2 + b3*b3 - 2*a2*b3* math.cos(theta3)
		
		# two halfs of angle beta
		gamma1 = math.acos( (k_sqrd + a2*a2 - b3*b3) / (2*math.sqrt(k_sqrd) * a2) )
		gamma2 = math.acos( (k_sqrd + b1*b1 - b2*b2) / (2*math.sqrt(k_sqrd) * b1) )
		
		# theta2 is difference of angles
		theta2 = theta1 - gamma1 - gamma2
		rospy.logdebug("recieved angles %f and %f, calculated angle %f", theta1, theta3, theta2)
		return theta2


	# Callback function for joint_state sub
	def _callback(self, virtJoints):
		# Extract angles
		theta1 = virtJoints.position[0]
		theta3 = virtJoints.position[3]
		
		# Calculate and set crank arm joint
		self._joints.position[1] = self.calcBeta(theta1,theta3)
		# Translate prismatic joint to mm
		self._joints.position[2] = 1000 * virtJoints.position[4]
		# Add last joint
		self._joints.position[0] = theta1
		# Add timestamp
		self._joints.header.stamp = rospy.Time.now()
		
		# While still running
		while not self.ctrl_c:
			# Check connections
			if self._pub.get_num_connections() > 0:
				self._pub.publish(self._joints)
				rospy.logdebug("Joint values published to Arduino")
				break
			else:
				rospy.logwarn("Could not publish calculated joint value")
				self.rate.sleep()


if __name__ == '__main__':
    try:
    	virt2phys_object = lift_arm_virt2phys()
    
    except rospy.ROSInterruptException:
    	pass
    rospy.spin()
