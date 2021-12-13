#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import time
from pickle import FALSE, FLOAT
from typing import get_type_hints
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
from Phidget22.Devices.Encoder import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Devices.CurrentInput import *


# Constants:
wheel_circumference = 505.0    		# mm
ticks_per_rev 		= 1200.0        # pulses per revolution (of the engine rotor shaft before gear)
wheelmount_radius 	= 185.0			# mm
trans_factor		= 18.0			# Transmission factor: wheel rpm / engine rotor rpm

# Global Variables:
setpointTwist = Twist()				# linear: [m/s]; angular: [rad/s]
v_setpoint_left 	= 0.0			# [m/s]
v_setpoint_right 	= 0.0			# [m/s]
dutycycle_set_left 	= 0.0			# [%] 1.0 = 100% / 0.0 = 0%
dutycycle_set_right = 0.0			# [%] 1.0 = 100% / 0.0 = 0%
v_sensor_left 		= 0.0			# [m/s]
v_sensor_right 		= 0.0			# [m/s]

#Create your Phidget channels
dcMotor0 			= DCMotor()
dcMotor1 			= DCMotor()
encoder0 			= Encoder()
encoder1 			= Encoder()
voltageRatioInput0 	= VoltageRatioInput()
voltageRatioInput1 	= VoltageRatioInput()
temperatureSensor0 	= TemperatureSensor()
temperatureSensor1 	= TemperatureSensor()
currentInput0 		= CurrentInput()
currentInput1 		= CurrentInput()



#Declare any event handlers here. These will be called every time the associated event occurs.
def onPositionChange0(self, positionChange, timeChange, indexTriggered):
	global v_sensor_left
	v_sensor_left = ticks_to_vel((-positionChange), timeChange, wheel_circumference, trans_factor, ticks_per_rev)

def onPositionChange1(self, positionChange, timeChange, indexTriggered):
	global v_sensor_right
	v_sensor_right = ticks_to_vel(positionChange, timeChange, wheel_circumference, trans_factor, ticks_per_rev)

def onVoltageRatioChange(self, voltageRatio):
	print("VoltageRatio: " + str(voltageRatio))

def onTemperatureChange(self, temperature):
	print("Temperature: " + str(temperature))

def onCurrentChange(self, current):
	print("Current: " + str(current))

def init_motorDriver():
	#Set addressing parameters to specify which channel to open (if any)
	dcMotor0.setHubPort(0)
	dcMotor0.setDeviceSerialNumber(635334)
	encoder0.setHubPort(0)
	encoder0.setDeviceSerialNumber(635334)
	voltageRatioInput0.setHubPort(0)
	voltageRatioInput0.setDeviceSerialNumber(635334)
	temperatureSensor0.setHubPort(0)
	temperatureSensor0.setDeviceSerialNumber(635334)
	currentInput0.setHubPort(0)
	currentInput0.setDeviceSerialNumber(635334)
	dcMotor1.setHubPort(1)
	dcMotor1.setDeviceSerialNumber(635334)
	encoder1.setHubPort(1)
	encoder1.setDeviceSerialNumber(635334)
	voltageRatioInput1.setHubPort(1)
	voltageRatioInput1.setDeviceSerialNumber(635334)
	temperatureSensor1.setHubPort(1)
	temperatureSensor1.setDeviceSerialNumber(635334)
	currentInput1.setHubPort(1)
	currentInput1.setDeviceSerialNumber(635334)

	#Assign any event handlers you need before calling open so that no events are missed.
	encoder0.setOnPositionChangeHandler(onPositionChange0)
	voltageRatioInput0.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
	temperatureSensor0.setOnTemperatureChangeHandler(onTemperatureChange)
	currentInput0.setOnCurrentChangeHandler(onCurrentChange)
	encoder1.setOnPositionChangeHandler(onPositionChange1)
	voltageRatioInput1.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
	temperatureSensor1.setOnTemperatureChangeHandler(onTemperatureChange)
	currentInput1.setOnCurrentChangeHandler(onCurrentChange)

	#Open your Phidgets and wait for attachment
	dcMotor0.openWaitForAttachment(500)				# [ms]
	encoder0.openWaitForAttachment(500)				# [ms]
	voltageRatioInput0.openWaitForAttachment(500)	# [ms]
	temperatureSensor0.openWaitForAttachment(500)	# [ms]
	currentInput0.openWaitForAttachment(500)		# [ms]
	dcMotor1.openWaitForAttachment(500)				# [ms]
	encoder1.openWaitForAttachment(500)				# [ms]
	voltageRatioInput1.openWaitForAttachment(500)	# [ms]
	temperatureSensor1.openWaitForAttachment(500)	# [ms]
	currentInput1.openWaitForAttachment(500)		# [ms]

	#Activate Fan for "Driver up and running" signaling
	# dcMotor0.setFanMode(True)
	# dcMotor1.setFanMode(True)

	#Set Position Acquisition Interval of Encoder
	encoder0.setDataInterval(100)		# [ms]
	encoder1.setDataInterval(100)		# [ms]

def terminate_motorDriver():
	#Closes all motor driver handles ordinarily and shuts down motors and feedbacks:
	dcMotor0.setTargetVelocity(0.0)
	dcMotor1.setTargetVelocity(0.0)
	dcMotor0.close()
	dcMotor1.close()
	encoder0.close()
	encoder1.close()	
	voltageRatioInput0.close()
	voltageRatioInput1.close()	
	temperatureSensor0.close()
	temperatureSensor1.close()	
	currentInput0.close()
	currentInput1.close()

	print("Phidgets Drive Units: Handles closed successfully!")

def limiter(dutycycle_val):	# [%]
	# Limits the duty cycle command value to a range between -1.0 ... 1.0. 
	limited_dutycycle_val = 0.0

	if (dutycycle_val > 1.0):
		limited_dutycycle_val = 1.0
	elif (dutycycle_val < -1.0):
		limited_dutycycle_val = -1.0
	else:
		limited_dutycycle_val = dutycycle_val

	return limited_dutycycle_val
	
# Transformations	
def ticks_to_vel(ticks, delta_t, wheel_circumference, transmissionfactor, ticks_per_rot): # [none], [ms], [mm], [rotor_revolution/wheel_revolution], [1/rotor_revolution]
	ticks_per_time = ticks/delta_t											# [1/ms]
	dist_per_ticks = wheel_circumference/(transmissionfactor*ticks_per_rot)	# [mm/1]

	vel_tangetial  = ticks_per_time * dist_per_ticks						# [mm/ms] = [m/s]						

	return vel_tangetial													# [m/s]

def diff_to_twist(v_left, v_right, wheelmount_radius):	# [m/s], [m/s], [mm]
	move = Twist()

	move.linear.x  = (v_right + v_left) / 2									# velocity of platform centerpoint
	move.angular.z = (v_right - v_left) / (wheelmount_radius*0.002) 		# angular velocity (omega)

	return move

def twist_to_diff(twistmsg, wheelmount_radius): 		# [m/s] resp. [rad/s], [mm]
	global v_setpoint_left
	global v_setpoint_right

	vel_tan_wheel = twistmsg.angular.z * (wheelmount_radius/1000)

	v_setpoint_left 	= twistmsg.linear.x - vel_tan_wheel
	v_setpoint_right	= twistmsg.linear.x + vel_tan_wheel



# Main Logic
def main():
	global v_setpoint_left
	global v_setpoint_right
	global dutycycle_set_left
	global dutycycle_set_right

	# Main Logic:
	dutycycle_set_left 	= v_setpoint_left
	dutycycle_set_right = v_setpoint_right

	# Command motor movement
	dcMotor0.setTargetVelocity(limiter(dutycycle_set_left))
	dcMotor1.setTargetVelocity(limiter(dutycycle_set_right))


# Subscriber callbacks:
def get_targetVel_left(data):
	global v_setpoint_left
	v_setpoint_left = data.data

def get_targetVel_right(data):
	global v_setpoint_right
	v_setpoint_right = data.data

def get_targetTwist_common(data):
	global setpointTwist
	global wheelmount_radius
	setpointTwist = data
	twist_to_diff(setpointTwist, wheelmount_radius)

# Subscribers:
#  subLeft_vel = rospy.Subscriber("motor/left/set_duty_cycle", Float64, get_targetVel_left)
#  subRight_vel = rospy.Subscriber("motor/right/set_duty_cycle", Float64, get_targetVel_right)
subCommon_twist = rospy.Subscriber("twist_mux/cmd_vel", Twist, get_targetTwist_common)

# Publishers:
pubLeft_ticks 	= rospy.Publisher('motor/left/sensor/ticks', Float64, queue_size=50)
pubRight_ticks 	= rospy.Publisher('motor/right/sensor/ticks', Float64, queue_size=50)
pubLeft_vel 	= rospy.Publisher('motor/left/sensor/is_vel', Float64, queue_size=50)
pubRight_vel	= rospy.Publisher('motor/right/sensor/is_vel', Float64, queue_size=50)
pubTwistFeedback = rospy.Publisher('motor/common/sensor/twist', Twist, queue_size=50)
feedback_twist 	= Twist()

# ROS: typical node initialisation, parametrization and program loop
rospy.init_node('drive_feedback', anonymous=False)
rate = rospy.Rate(20) 									# 20hz

if __name__ == '__main__':
	init_motorDriver()

	try:
		while not rospy.is_shutdown():
			main()									# Main Program Logic
			pubLeft_vel.publish(v_sensor_left)
			pubRight_vel.publish(v_sensor_right)
			pubTwistFeedback.publish(diff_to_twist(v_sensor_left, v_sensor_right, wheelmount_radius))
			rate.sleep()
		
		rospy.spin()								# Avoid python from closing the ROS node while beeing up and executed
		rospy.on_shutdown(terminate_motorDriver())	# Terminate hardware functions of native "Phidgets Drivers" driver (Python)
	except rospy.ROSInterruptException:
		pass