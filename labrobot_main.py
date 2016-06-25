import sys
import time

from array import *

spacebrew_path = '/home/pi/LAB_ROBOT/pySpacebrew'
roboclaw_path = '/home/pi/LAB_ROBOT/roboclaw_python'

sys.path.append(roboclaw_path)
sys.path.append(spacebrew_path)

import roboclaw
from pySpacebrew.spacebrew import Spacebrew

# get app name and server from query string
name = "Lab Robot - Test Controls"
_server = "192.168.1.143"; #sandbox.spacebrew.cc"

# configure the spacebrew client
brew = Spacebrew(name, server=_server)

brew.addSubscriber("motorA", "range")
brew.addSubscriber("motorB", "range")

MOTOR_MIN = 0
MOTOR_MAX = 70

_valMotor = array('i',[0,0])
_targetMotor = array('i',[0,0])

#for i in range(2):
#	_valMotor[i] = 0
#	_targetMotor[i] = 0

# function that handles the incoming spacebrew range messages
def motor1(_val):

	global _targetMotor

	_targetMotor[0] = mapValues(int(_val), 0, 1023, -MOTOR_MAX, MOTOR_MAX)
        print "t: " + str(_targetMotor[0])

def motor2(_val):

	global _targetMotor

	_targetMotor[1] = mapValues(int(_val), 0, 1023, -MOTOR_MAX, MOTOR_MAX)
        print "t: " + str(_targetMotor[1])

# Map Values
def mapValues(in_val, in_from, in_to, out_from, out_to):

    _val = 0

    out_range = out_to - out_from
    in_range = in_to - in_from
    in_val = in_val - in_from
    _val = (in_val*out_range) / in_range
    out_val = out_from + _val

    return out_val

# registering range handler method with appropriate subscription feed
brew.subscribe("motorB", motor2)
brew.subscribe("motorA", motor1)

#Linux comport name
roboclaw.Open("/dev/ttyACM0",115200)

address = 0x80
_val = 0
_alpha = 2
_speed = _alpha
_stage = 0

try:
	brew.start()

	while(1):

		#FOR BOTH MOTORS
		for i in range(2):
			_diff = _targetMotor[i] - _valMotor[i]

			if abs(_diff) > _alpha:
			
				if _diff > 0:
					_valMotor[i] = _valMotor[i] + _alpha	
				else:
					_valMotor[i] = _valMotor[i] - _alpha	

			print "tar " + str(i) + ": " + str(_targetMotor[i]) + " val : " + str(_valMotor[i])

			if i == 0:
				if _valMotor[i] > 0:
		 			roboclaw.ForwardM1(address,_valMotor[i])
				else:
					roboclaw.BackwardM1(address,abs(_valMotor[i]))
			else:
				if _valMotor[i] > 0:
		 			roboclaw.ForwardM2(address,_valMotor[i])
				else:
					roboclaw.BackwardM2(address,abs(_valMotor[i]))

		time.sleep(.03)

finally:
	brew.stop()

