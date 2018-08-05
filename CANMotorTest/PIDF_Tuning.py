#!/usr/bin/env python

import rospy
from titan_base.msg import *
import numpy
from matplotlib import pyplot as plt


def cbMotorStatus(status):
	dev_str = ''
	if (status.DeviceId == 1):
		dev_str = 'LEFT'
	if (status.DeviceId == 4):
		dev_str = 'RIGHT'
	if (status.DeviceId == 1 or status.DeviceId == 4):
		rospy.loginfo(dev_str + " - Voltage: %f - CloseLoopErr: %i - Pos %i - Vel %i - Throttle: %i", status.DeviceId, status.BatteryV, status.CloseLoopErr,status.SensorPosition,status.SensorVelocity,status.AppliedThrottle)

def PIDFTuning():
	rospy.init_node('PIDFTuning', anonymous=True)
	r = rospy.Rate(50)

	rospy.Subscriber("/motor_status", Status, cbMotorStatus)

	while not rospy.is_shutdown():
		r.sleep()

if __name__ == '__main__':
	PIDFTuning()
