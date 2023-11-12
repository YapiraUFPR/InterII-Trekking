#!/usr/bin/env python
import sys, select, termios, tty
import board
import pwmio
from adafruit_motor import servo
from tests.motors.esc import Esc
from time import sleep

ESC_PIN = 14
SERVO_PIN = board.D18

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main():	

	speed = 0.0
	turn = 0.2
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	# esc_pwm = pwmio.PWMOut(ESC_PIN, frequency=100)
	# esc = servo.ContinuousServo(esc_pwm, min_pulse=500, max_pulse=2500)
	# esc.throttle = 0
	esc =  Esc(ESC_PIN)
	servo_pwm = pwmio.PWMOut(SERVO_PIN, frequency=50)
	servo_motor = servo.Servo(servo_pwm, min_pulse=750, max_pulse=2250)
	servo_motor.angle = 90

	angle =  90

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			# twist = Twist()
			# twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			# twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
			# pub.publish(twist)
			linear_x = x*speed
			angular_z = th*turn

			angle += -angular_z * 90
			angle = int(max(0, min(angle, 180)))

			mspeed = linear_x * 100
			mspeed = int(max(-100, min(mspeed, 100)))

			print(mspeed, angle)

			esc.set_speed(mspeed)
			servo_motor.angle = angle

	except Exception as e:
		print(e)

	finally:
		# twist = Twist()
		# twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		# twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		# pub.publish(twist)
		esc.speed = 0
		servo_motor.angle = 0

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
	main()