#!/usr/bin/env python

import pdb
import rospy
import math
from sensor_msgs.msg import *
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import *
from geometry_msgs.msg import Twist
import numpy as np
from numpy.linalg import inv
import lp_filter as lp
import med_filter as mdf
import pdb
#TODO: 
#- some basic commands like reset position and so on
# test if control works
# add arm position calibration in this script for J1 and J2. see embedded code for manual setting via ros message.


class arm_controller():

	def __init__(self, isleft = True):
		self.sync_time = 1.0/10.0
		self.Hz = 50.0
		self.rate = rospy.Rate(self.Hz) 
		self.isleft = isleft
		self.G_RATIO1 = 12.18
		self.G_RATIO2 = 12.18
		self.joint2_limit = 1.25
		self.speed_limit_J1 = 0.5
		self.speed_limit_J2 = 0.5
		self.last_sync = rospy.get_time()

		self.st = 'left'

		if self.isleft:
			self.st = 'left'
		else:
			self.st = 'right'

		#Publisher
		self.pubMcmd_Arm = rospy.Publisher('/quori/arm_' + self.st + '/cmd_pos_dir', Vector3, queue_size=1)
        # x: Motor position command in radians. This command is meant for the outer motor in the port nearest to the USB. 
        # y: Motor position command in radians. This command is meant for the inner motor in the port away from the USB.
        # z: varible that can be used for timeing. Sending -1 leads to a coast command
		self.pubJointState = rospy.Publisher('/quori/arm_' + self.st + '/shoulder_pos', Vector3, queue_size=1)
        # x: Joint position in radians. This is meant to represent the circumduction motion and is refered to as J1. The sensor should be pluged into the port furtherest from the USB port on the microcontroller.
		# y: Joint position in radians. This is meant to represent the addduction and abduction motion and is refered to as J2. The sensor should be pluged into the port nearest to the USB port on the microcontroller. 
		# z: unused
		self.pubJointSpeed = rospy.Publisher('/quori/arm_' + self.st + '/shoulder_speed', Vector3, queue_size=1)

		

		#state
		self.mode = 0 # mode of operation. 0: coast, 1: allow motion

		self.J1nWraps = 0

		self.arm_callibration_offset1 = 0.0 #TODO: create a way to set this via the parameter server
		self.arm_callibration_offset2 = -0.0 #TODO: create a way to set this via the parameter server



		self.motor_cmd     = Vector3() #motor commands loaded to this
		self.ArmMsg        = Vector3() #arm goal positions loaded to this
		self.pubJointState_msg = Vector3() #arm updated position considering wrapping
		self.pubJointSpeed_msg = Vector3()
		self.armMotorPosmeas = Vector3()
		self.armJointPosmeas = Vector3()
		self.armJointPosRawmeas = Vector3()
		self.armJointPos_sync = Vector3()
		self.armJointPosRawmeasPrev = Vector3()
		self.last_pubJointState_msg = Vector3()
		self.motorDriftSync = Vector3()

		self.motorDriftSync.x = 0.0
		self.motorDriftSync.y  = 0.0
		self.armJointPos_sync.x = 0.0
		self.armJointPos_sync.y = 0.0

	    
		self.armJointPosRawmeas.x  = 0.0
		self.armJointPosRawmeas.y  = 0.0
		self.armJointPosRawmeasPrev.x  = 0.0
		self.armJointPosmeas.x = 0.0
		self.armJointPosmeas.y = 0.0
		self.last_pubJointState_msg.x = 0.0
		self.last_pubJointState_msg.y = 0.0

		self.lp_filterx = lp.lp_filter()
		self.lp_filterx.set_const(0.175) # between 0.25 and 0.1
		self.lp_filtery = lp.lp_filter()
		self.lp_filtery.set_const(0.175)
		self.med_filtery = mdf.med_filter(3)
		self.position_error_limit = 0.5
		self.arm_sensor_inited = False
		self.arm_motor_inited = False
		self.arm_motor_updated = [False, False]
		self.arm_joint_updated = [False, False]
		self.rollOverUB = 3.0 
		self.rollOverLB = -3.0 
		self.joint2_LB = 0.7

		# Value greater than which sensor reading is considered garbage and is rejected
		self.y_joint_unreasnable_lim = 1.6 #1.4 or 1.5 are also not feasible with panels on


	def subscribe2topics(self):
		#Subscriber 
		rospy.Subscriber('/quori/arm_' + self.st + '/pos_status',Vector3,self.armJ_callback)
		# x: Raw Joint position in radians. This is meant to represent the circumduction motion and is refered to as J1. The sensor should be pluged into the port furtherest from the USB port on the microcontroller.
		# y: Raw Joint position in radians. This is meant to represent the addduction and abduction motion and is refered to as J2. The sensor should be pluged into the port nearest to the USB port on the microcontroller. 
		# z: unused.
		rospy.Subscriber('/quori/arm_' + self.st + '/motor_status',Vector3,self.armM_callback)
		# x: Motor position in radians. This is read from the microcontroller and is meant to describe the outer motor and is refered to as M1. The outer motor is the one plugged into the port near tothe USB. 
        # y: Motor position in radians. This is read from the microcontroller and is meant to describe the inner motorand is refered to as M2. The inner motor is the one plugged into the port away from the USB. 
        # z: unused
		rospy.Subscriber('/quori/arm_' + self.st + '/joint_goal',Vector3,self.goal_callback) #TODO create a script that publishes this information

	
	def goal_callback(self,data):

		if data.z <0.0:
			self.mode = 0
			self.coast_msg()
		else:
			self.ArmMsg.x = data.x
			self.ArmMsg.y = data.y
			self.ArmMsg.z = data.z
			self.limit_check() #adjust input if nessesary
			self.mode = 1
		
		#rospy.loginfo('goal received')
		


	def armJ_callback(self,msg):
		#shift by a potential offset due to hardware chage. This can also be done in the embedded code.
		self.armJointPosRawmeas.x = msg.x
		self.armJointPosRawmeas.y = msg.y

		# Filter impossible readings in y sensor of left arm
		previous_value = self.armJointPosmeas.y 
		error_speed = abs(self.armJointPosRawmeas.y-previous_value)
		if abs(self.armJointPosRawmeas.y) > self.y_joint_unreasnable_lim:
			self.armJointPosmeas.y = self.armJointPosmeas.y
			rospy.logwarn('error-reasonable value received for y:'+self.st)
		elif error_speed>self.position_error_limit:
			if self.arm_sensor_inited:
				self.armJointPosmeas.y = self.armJointPosmeas.y
				rospy.logwarn('error-reasonable speed sensed for y:'+self.st)
			else: #assume the arm just turned on and the data is useful
				self.armJointPosmeas.y = self.armJointPosRawmeas.y
		else: #probably good data
			if not self.med_filtery.inited:
				self.med_filtery.init_filter(self.armJointPosRawmeas.y)
			self.armJointPosmeas.y = self.med_filtery.ComputeFrmNew(self.armJointPosRawmeas.y)

		self.handle_J1_wrap()

		self.pubJointState_msg.x = self.armJointPosmeas.x
		self.pubJointState_msg.y = self.armJointPosmeas.y

		self.pubJointSpeed_msg.x = (self.last_pubJointState_msg.x - self.armJointPosmeas.x)*self.Hz
		self.pubJointSpeed_msg.y = (self.last_pubJointState_msg.y - self.armJointPosmeas.y)*self.Hz
		self.last_pubJointState_msg.x = self.armJointPosmeas.x
		self.last_pubJointState_msg.y = self.armJointPosmeas.y
		if not self.arm_sensor_inited:
			self.arm_sensor_inited = True
			self.sync_pos()
			rospy.loginfo('arm sensor read. init arm J')


	def armM_callback(self,msg):
		if msg.z == 3.0:
			self.armMotorPosmeas.x = msg.x
			self.arm_motor_updated[0] = True
			self.armMotorPosmeas.y = msg.y
			self.arm_motor_updated[1] = True
		elif  msg.z == 2.0:
			self.armMotorPosmeas.y = msg.y
			self.arm_motor_updated[0] = False
			self.arm_motor_updated[1] = True
		elif msg.z == 1.0:
			self.armMotorPosmeas.x = msg.x
			self.arm_motor_updated[0] = True
			self.arm_motor_updated[1] = False
		else: 
			self.arm_motor_updated[0] = False
			self.arm_motor_updated[1] = False

		if not self.arm_motor_inited and msg.z == 3.0: 
			self.arm_motor_inited = True
			self.sync_pos()
			rospy.loginfo('motors sensed on.')
			if not self.lp_filterx.inited and not self.lp_filtery.inited:
				self.lp_filterx.init_filter(msg.x)
				self.lp_filtery.init_filter(msg.y)
				rospy.loginfo("inited arm motor pos filter")
				rospy.loginfo(self.st + ":"+ str(self.lp_filterx.old))
				rospy.loginfo(self.st + ":" + str(self.lp_filterx.new))

	def coast_msg(self):
		self.motor_cmd.x = self.motor_cmd.x #just give it the last recieved command to preserve it.
		self.motor_cmd.y = self.motor_cmd.y
		self.motor_cmd.z = -1.0

	def limit_check(self):
		#protect the arms from moving too fast
		#change so the vector direction is preserved but scaled by the fastest desired speed
		# speed modification is desabled in this version of the code.

		if abs(self.armJointPosmeas.x-self.ArmMsg.x)>self.speed_limit_J1:
			#self.ArmMsg.x = self.armJointPosmeas.x+self.speed_limit_J1*math.copysign(1, (self.ArmMsg.x-self.armJointPosmeas.x))
			rospy.logwarn('Joint speed limit x for ' +self.st + ' reached')
		else:
			self.ArmMsg.x = self.ArmMsg.x
		if abs(self.armJointPosmeas.y-self.ArmMsg.y)>self.speed_limit_J2:
			#self.ArmMsg.y = self.armJointPosmeas.y+self.speed_limit_J2*math.copysign(1, (self.ArmMsg.y-self.armJointPosmeas.y))
			rospy.logwarn('Joint speed limit y for ' + self.st + ' reached')
		else:
			self.ArmMsg.y = self.ArmMsg.y


		#TODO: protect the motors from moving too fast
		
		
		#protect joint 2
		if not self.isleft and (self.ArmMsg.y > self.joint2_limit):
			self.ArmMsg.y = self.joint2_limit
			#rospy.logwarn('Joint limit for ' + self.st + ' reached')
		elif self.isleft and (self.ArmMsg.y < -self.joint2_limit):
			self.ArmMsg.y = -self.joint2_limit
			#rospy.logwarn('Joint limit for ' + self.st + ' reached'

	def setMotorGoalFromArm(self):
		if self.arm_sensor_inited and self.arm_motor_inited:
			diff_x = self.ArmMsg.x-self.armJointPos_sync.x
			diff_y = self.ArmMsg.y-self.armJointPos_sync.y
			self.motor_cmd.z = 1.0/self.Hz
			self.motor_cmd.x = self.G_RATIO1 * (diff_x + diff_y)+self.motorDriftSync.x
			#print 'pre: '
			#print self.motor_cmd.x
			#pdb.set_trace()
			#rospy.logwarn("arm warn test")
			self.motor_cmd.y = self.G_RATIO2 * (diff_x - diff_y)+self.motorDriftSync.y
			if self.lp_filterx.inited and self.lp_filtery.inited:
				#rospy.logwarn("filtering")
				# print "filtering"
				self.lp_filterx.set_new(self.motor_cmd.x)
				self.lp_filtery.set_new(self.motor_cmd.y)
				self.motor_cmd.x = self.lp_filterx.FilterCompute()
				self.motor_cmd.y = self.lp_filtery.FilterCompute()
		else:
			self.mode = 0 #send coast until we get all the data we need
			self.motor_cmd.z = -1 #send coast until we get all the data we need
			#print 'waiting to get data'
			#rospy.logwarn("arm not init_ Sending coast")

				

	def sync_pos(self):
		self.motorDriftSync.x = self.armMotorPosmeas.x
		self.motorDriftSync.y  = self.armMotorPosmeas.y

		self.armJointPos_sync.x = self.armJointPosmeas.x
		self.armJointPos_sync.y = self.armJointPosmeas.y


	def handle_J1_wrap(self):
		#if wrap detected increment number of wraps then add measurement to it. if 
		diff = self.armJointPosRawmeas.x-self.armJointPosRawmeasPrev.x 
		if abs(diff)> math.pi:
			if diff < 0:
				self.J1nWraps = self.J1nWraps + 1
			elif diff >= 0:
				self.J1nWraps = self.J1nWraps - 1
		#add wraps if needed
		if self.J1nWraps ==1:
			self.armJointPosmeas.x = math.pi+(math.pi+self.armJointPosRawmeas.x)
		elif self.J1nWraps ==-1:
			self.armJointPosmeas.x = -math.pi-(math.pi-self.armJointPosRawmeas.x)
		elif self.J1nWraps >1:
			self.armJointPosmeas.x = math.pi+(self.J1nWraps-1)*math.pi*2.0+(math.pi+self.armJointPosRawmeas.x)
		elif self.J1nWraps <-1:
			self.armJointPosmeas.x = -math.pi+(self.J1nWraps+1)*math.pi*2.0-(math.pi-self.armJointPosRawmeas.x)
		elif self.J1nWraps == 0:
			self.armJointPosmeas.x = self.armJointPosRawmeas.x
		self.armJointPosRawmeasPrev.x = self.armJointPosRawmeas.x


if __name__ == '__main__':
	rospy.init_node('Arm_Controller_node')
	rospy.loginfo('Arm Controller node started')
	controller_right = arm_controller(0)# change to 0 later
	controller_left = arm_controller(1)	
	controller_right.subscribe2topics()
	controller_left.subscribe2topics()
	#Sync before testing
	#controller.sync_pos()

	while not rospy.is_shutdown():
		#Right Arm
		if controller_right.mode == 1:
			controller_right.setMotorGoalFromArm()
		elif controller_right.mode == 0:
			#coast
			controller_right.coast_msg()
		# Left Arm 
		if controller_left.mode == 1:
			controller_left.setMotorGoalFromArm()
		elif controller_left.mode == 0:
			#coast
			controller_left.coast_msg()
		#publish commands

		controller_left.pubMcmd_Arm.publish(controller_left.motor_cmd) 
		controller_left.pubJointState.publish(controller_left.pubJointState_msg)#TODO. change to ROS joint state message
		controller_left.pubJointSpeed.publish(controller_left.pubJointSpeed_msg)

		#publish commands
		controller_right.pubMcmd_Arm.publish(controller_right.motor_cmd) 
		controller_right.pubJointState.publish(controller_right.pubJointState_msg)#TODO. change to ROS joint state message
		controller_right.pubJointSpeed.publish(controller_right.pubJointSpeed_msg)

		if (rospy.get_time()-controller_right.last_sync)>controller_right.sync_time and 1:
			controller_right.sync_pos()
			controller_left.sync_pos()
			controller_right.last_sync = rospy.get_time()
		controller_right.rate.sleep()	