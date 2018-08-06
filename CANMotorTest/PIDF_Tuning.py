#!/usr/bin/env python

import rospy
from titan_base.msg import *
import numpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import sys

class PIDFTuning():

	def __init__(self):
		style.use('fivethirtyeight')
		plt.close('all')
		self.fig, self.axarr = plt.subplots(4,sharex=True)

		rospy.init_node('PIDFTuning', anonymous=True)
		
		self.recording = True
		self.time_pts = []
		self.error_pts = []
		self.throttle_pts = []
		self.pos_pts = []
		self.vel_pts = []
	
		rospy.Subscriber("/motor_status", Status, self.cbMotorStatus)
	

		self.vel_pub = rospy.Publisher('/motor_velocity', MotorVelocity, queue_size=10)
		self.pidf_pub = rospy.Publisher('/set_pidf_param', PIDF, queue_size=10)

		pidf = PIDF()
		pidf.P_Gain = 1.6
		pidf.I_Gain = 0.001
		pidf.D_Gain = 30.0
		pidf.F_Gain = 1.0
		r = rospy.Rate(5)
		r.sleep()
		self.pidf_pub.publish(pidf)
		self.start_time = rospy.get_rostime()
		
	def cbMotorStatus(self,status):
		dev_str = ''
	
		if (status.DeviceId == 1):
			dev_str = 'LEFT'
		if (status.DeviceId == 4):
			dev_str = 'RIGHT'
		timestamp = status.header.stamp.secs + (status.header.stamp.nsecs / 1.0e9)
		if (status.DeviceId == 1):
			#
			if (self.recording == True):
				rospy.loginfo(str(timestamp) + " - " + dev_str + " - Voltage: %f - CloseLoopErr: %i - Pos %i - Vel %i - Throttle: %i", status.BatteryV, status.CloseLoopErr,status.SensorPosition,status.SensorVelocity,status.AppliedThrottle)
				
				#recorded_data.append( (timestamp, status.CloseLoopErr, status.AppliedThrottle, status.SensorPosition, status.SensorVelocity))
				self.time_pts.append(timestamp)
				self.error_pts.append(status.CloseLoopErr)
				self.throttle_pts.append(status.AppliedThrottle)
				self.pos_pts.append(status.SensorPosition)
				self.vel_pts.append(status.SensorVelocity)
	
	def compareVel(self):
		for i in range(len(self.time_pts)):
			if (i > 0):
				deltaTime = self.time_pts[i] - self.time_pts[i-1]
				deltaEncoder = self.pos_pts[i] - self.pos_pts[i-1]
				deltaRadian = deltaEncoder / 1024.0 / (2*3.1415);


				enc_vel = (deltaEncoder / deltaTime) / 20.0;
				rad_vel = deltaRadian / deltaTime
				print "dt: " + str(deltaTime) + " dEncoder: " + str(deltaEncoder) + " dRadian: " + str(deltaRadian)
				print "Calc: " + str(enc_vel) + " tick/100 ms -- Sensor: " + str(self.vel_pts[i])
				print "Rad/s: " + str(rad_vel)

	def Main(self):
		d = rospy.Duration.from_sec(3.0)
		r = rospy.Rate(50)
		while not rospy.is_shutdown():
			cur_time = rospy.get_rostime()
			if (cur_time - self.start_time <= d):
				motor_cmd = MotorVelocity()
				motor_cmd.left_angular_vel = 300;
				motor_cmd.right_angular_vel = 0;
				self.vel_pub.publish(motor_cmd);
			else:
				self.recording = False
				#self.compareVel()
				
				self.axarr[0].clear()
				self.axarr[1].clear()
				self.axarr[2].clear()
				self.axarr[3].clear()

				self.axarr[0].plot(self.time_pts,self.error_pts)
				self.axarr[1].plot(self.time_pts,self.throttle_pts)
				self.axarr[2].plot(self.time_pts,self.pos_pts)
				self.axarr[3].plot(self.time_pts,self.vel_pts)
				plt.show()
				#print self.pos_pts
				return
			r.sleep()

	#print "test"

if __name__ == '__main__':
	p = PIDFTuning()
	p.Main()
	#if sys.flags.interactive != 1 or not hasattr(pg.QtCore, 'PYQT_VERSION'):
	#	pg.QtGui.QApplication.exec_()
