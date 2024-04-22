#!/usr/bin/env python

import serial
import time
import rospy
from std_msgs.msg import String
import string

printable = string.ascii_letters + string.digits + string.punctuation + ' '
def hex_escape(s):
	return ''.join(c if c in printable else r'\x{0:02}'.format(ord(c)) for c in s)

def syncinfo(data):
	command = data.data
	command = hex_escape(command)
	arduino.write(bytes(command.encode('utf-8')))
	time.sleep(0.05)
	
	
	
if __name__ == "__main__":
	try:
		arduino = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=20)
	except:
		arduino = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=20)
	rospy.init_node("serial_communicator")
	rospy.Subscriber("/serial_msgs", String, syncinfo)
	rospy.spin()
 
 
# import serial
# import time

# arduino = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=20)

# def write_read(x):
# 		arduino.write(bytes(x, 'utf-8'))
# 		time.sleep(0.05)
		# data = arduino.readline()
		# return data

# while True:
# 	num = input("Enter the command in the form of 'number, meters, turns'")
# 	value = write_read(num)
	## print(value)