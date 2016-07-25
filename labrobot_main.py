from socketIO_client import SocketIO, LoggingNamespace

import asyncio
from Tkinter import *
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

pwm = GPIO.PWM(18, 100)
pwm.start(5)

def update(self, angle):
  duty = float(angle) / 10.0 + 2.5
  pwm.ChangeDutyCycle(duty)

headAngle = 90

import smbus
import sys
import time
import curses

#import RPi.GPIO as GPIO
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(11,GPIO.OUT)

#pwm=GPIO.PWM(11,50)
#pwm.start(5)

#_duty = 1/18*(180) + 2
#pwm.ChangeDutyCycle(_duty)

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

from array import *

spacebrew_path = '/home/pi/LAB_ROBOT/pySpacebrew'
roboclaw_path = '/home/pi/LAB_ROBOT/roboclaw_python'

sys.path.append(roboclaw_path)
sys.path.append(spacebrew_path)

import roboclaw
#from pySpacebrew.spacebrew import Spacebrew

#stdscr = curses.initscr()
#curses.cbreak()
#stdscr.keypad(1)
#stdscr.refresh()

_valMotor = array('i',[0,0])
_targetMotor = array('i',[0,0])

_moveType = 0
_running = True
_speedAlpha = 5

moving = False

# CHANGE SERVER TO CORRECT

#Adam Home
#socketIO = SocketIO('192.168.8.101', 5000, LoggingNamespace)

#Adam Lab
socketIO = SocketIO('192.168.1.143', 5000, LoggingNamespace)

# MOTOR VALUES
_speedAlpha = 5

def writeToArduino(val):

  for character in str(val):
    print(character)
    bus.write_byte(address, ord(character))

  return -1

######################
# Get control from socket
########################
def getStick1(*args):
 
  global moving 
  _v = "3" + str(args[0])

  print(_v)
  writeNumber(int(_v))
 
  #if moving == False:
  #  moving = True
  #  writeToArduino(_v)
  #  time.sleep(1)

def getStick2(*args):
  
  _v = "1" + str(args[0])
  writeNumber(_v)

def getUp(*args):

  print("Up")

  initDir(0)
  addSpeed(0,_speedAlpha)
  addSpeed(1,_speedAlpha)
  
def getDown(*args):

  print("Down")

  initDir(0)
  addSpeed(0,-_speedAlpha)
  addSpeed(1,-_speedAlpha)
  
def getLeft(*args):

  initDir(1)
  addSpeed(0,-_speedAlpha)
  addSpeed(1,_speedAlpha)
  
def getRight(*args):
  
  initDir(1)
  addSpeed(0,_speedAlpha)
  addSpeed(1,-_speedAlpha)

def getTrayUp(*args):
  
  _v = "1001"
  writeNumber(_v)

def getTrayDown(*args):
  
  _v = "1002"
  writeNumber(_v)

def initDir(_tMoveType):
  
  global _moveType, _targetMotor

  if _tMoveType != _moveType:

    _moveType = _tMoveType

    _targetMotor[0] = 0
    _targetMotor[1] = 0

def addSpeed(_i, _s):

  global _targetMotor, MOTOR_MAX

  _t = _targetMotor[_i] + _s

  if _t > MOTOR_MAX:
    _t = MOTOR_MAX

  if _t < -MOTOR_MAX:
    _t = -MOTOR_MAX

  _targetMotor[_i] = int(_t)

_k = ''
_motor0 = 0;
_motor1 = 0;

_keySpeed = 0.0
_keyTurn = 0.0


#################
# SOCKET LOOP
#################
@asyncio.coroutine
def socketLoop():
  socketIO.wait(.1)
  asyncio.async(socketLoop())


##################
# Set Motor Loops
#################
@asyncio.coroutine
def motorLoop():

  _k = -1; #stdscr.getch()
  if _k != -1:
    print(str(_k))

    if _k == ord('i'):

      initDir(0)

      addSpeed(0,_speedAlpha)
      addSpeed(1,_speedAlpha)

    if _k == ord('k'):

      initDir(0)

      addSpeed(0,-_speedAlpha)
      addSpeed(1,-_speedAlpha)

    if _k == ord('l'):

      initDir(1)

      addSpeed(0,_speedAlpha)
      addSpeed(1,-_speedAlpha)

    if _k == ord('j'):

      initDir(1)

      addSpeed(0,-_speedAlpha)
      addSpeed(1,_speedAlpha)

  #print str(_targetMotor[0]) + ' ' + str(_targetMotor[1])
  setMotors()

  asyncio.async(motorLoop())


# get app name and server from query string
name = "Lab Robot - Test Controls"
#_server = "192.168.1.143"; #sandbox.spacebrew.cc"

MOTOR_MIN = 0
MOTOR_MAX = 70

ENABLE_MOTORS = True

#
# Write To Arduino
#
def writeToArduino(_val):

  for character in str(_val):
    print(character)
    #bus.write_byte(address, ord(character))

  return -1

#for i in range(2):
#	_valMotor[i] = 0
#	_targetMotor[i] = 0

# function that handles the incoming spacebrew range messages
def motor1(_val):

  global _targetMotor

  _targetMotor[0] = mapValues(int(_val), 0, 1023, -MOTOR_MAX, MOTOR_MAX)
  print("t: " + str(_targetMotor[0]))

def motor2(_val):

  global _targetMotor

  _targetMotor[1] = mapValues(int(_val), 0, 1023, -MOTOR_MAX, MOTOR_MAX)
  print("t: " + str(_targetMotor[1]))

# Map Values
def mapValues(in_val, in_from, in_to, out_from, out_to):

    _val = 0

    out_range = out_to - out_from
    in_range = in_to - in_from
    in_val = in_val - in_from
    _val = (in_val*out_range) / in_range
    out_val = out_from + _val

    return out_val


address = 0x80
_val = 0
_alpha = 5
_speed = _alpha
_stage = 0

IS_KEYBOARD = True
IS_SOCKETS = False

def setMotors():

	global _targetMotor, _valMotor, ENABLE_MOTORS

	#FOR BOTH MOTORS
	for i in range(2):

		_diff = _targetMotor[i] - _valMotor[i]

		if abs(_diff) >= _alpha:

			if _diff > 0:
				_valMotor[i] = _valMotor[i] + _alpha	
			else:
				_valMotor[i] = _valMotor[i] - _alpha	

			if ENABLE_MOTORS == True: 
				if i == 0:
					if _valMotor[i] > 0:
	 					roboclaw.ForwardM1(address,_valMotor[i])
					else:
						roboclaw.BackwardM1(address,abs(_valMotor[i]))
				else:
					if _valMotor[i] > 0:
						roboclaw.BackwardM2(address,_valMotor[i])
					else:
		 				roboclaw.ForwardM2(address,abs(_valMotor[i]))	

			time.sleep(.03)

	#print ">" + str(_valMotor[0]) + " " + str(_valMotor[1])

#
# BEGIN HERE
# 
loop = asyncio.get_event_loop()

try:

  #Linux comport name
  if ENABLE_MOTORS == True:
    roboclaw.Open("/dev/ttyACM0",115200)

  #Listen to the Socket
  socketIO.on('stick1', getStick1) 
  socketIO.on('stick2', getStick2) 
  socketIO.on('buttonUp', getUp) 
  socketIO.on('buttonDown', getDown) 
  socketIO.on('buttonLeft', getLeft) 
  socketIO.on('buttonRight', getRight) 
  socketIO.on('buttonTrayUp', getTrayUp) 
  socketIO.on('buttonTrayDown', getTrayDown) 
 
  asyncio.async(socketLoop())
  asyncio.async(motorLoop())

  loop.run_forever()
 
except KeyboardInterrupt:
  pass

finally:
  loop.close()

