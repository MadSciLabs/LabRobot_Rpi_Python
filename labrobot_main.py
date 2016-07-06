from socketIO_client import SocketIO, LoggingNamespace

import sys
import time
import curses

from array import *

spacebrew_path = '/home/pi/LAB_ROBOT/pySpacebrew'
roboclaw_path = '/home/pi/LAB_ROBOT/roboclaw_python'

sys.path.append(roboclaw_path)
sys.path.append(spacebrew_path)

#import roboclaw
#from pySpacebrew.spacebrew import Spacebrew

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
stdscr.refresh()

_valMotor = array('i',[0,0])
_targetMotor = array('i',[0,0])

_moveType = 0
_running = True

# CHANGE SERVER TO CORRECT
socketIO = SocketIO('192.168.8.102', 5000, LoggingNamespace)

def writeNumber(value):
    bus.write_byte(address, value)
    # bus.write_byte_data(address, 0, value)
    return -1

######################
# Get control from socket
########################
def getStick1(*args):
  
  _v = "3" + str(args[0])

  print _v
  print int(_v)
  writeNumber(int(_v))

def getStick2(*args):
  
  _v = "1" + str(args[0])
  writeNumber(int(_v))

def getUp(*args):

  initDir(0)
  addSpeed(0,_speedAlpha)
  addSpeed(1,_speedAlpha)
  
def getDown(*args):

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
  writeNumber(int(_v))

def getTrayDown(*args):
  
  _v = "1002"
  writeNumber(int(_v))

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


def main(stdscr):

  global _targetMotor
  global _moveType

  _speedAlpha = 5

  _k = ''
  _motor0 = 0;
  _motor1 = 0;

  _keySpeed = 0.0
  _keyTurn = 0.0

  stdscr.nodelay(1)

  #Listen to the Socket
  socketIO.on('stick1', getStick1) 
  socketIO.on('stick2', getStick2) 
  socketIO.on('buttonUp', getUp) 
  socketIO.on('buttonDown', getDown) 
  socketIO.on('buttonLeft', getLeft) 
  socketIO.on('buttonRight', getRight) 
  socketIO.on('buttonTrayUp', getTrayUp) 
  socketIO.on('buttonTrayDown', getTrayDown) 

  socketIO.wait()

  while True:
	
    _k = stdscr.getch()
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


# get app name and server from query string
name = "Lab Robot - Test Controls"
_server = "192.168.1.143"; #sandbox.spacebrew.cc"

# configure the spacebrew client
#brew = Spacebrew(name, server=_server)

#brew.addSubscriber("motorA", "range")
#brew.addSubscriber("motorB", "range")

MOTOR_MIN = 0
MOTOR_MAX = 70

ENABLE_MOTORS = False


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
#brew.subscribe("motorB", motor2)
#brew.subscribe("motorA", motor1)

#Linux comport name
if ENABLE_MOTORS == True:
    roboclaw.Open("/dev/ttyACM0",115200)

address = 0x80
_val = 0
_alpha = 5
_speed = _alpha
_stage = 0

IS_KEYBOARD = True
IS_SOCKETS = False

if IS_SOCKETS == True:

	try:
		brew.start()
	finally:
		brew.stop()

#Set data
#def setSpeed():

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

	print ">" + str(_valMotor[0]) + " " + str(_valMotor[1])

if __name__ == '__main__':
	curses.wrapper(main)

curses.endwin()

