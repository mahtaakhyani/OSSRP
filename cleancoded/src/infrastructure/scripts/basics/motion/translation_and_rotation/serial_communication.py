import serial
import time

arduino = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=20)

def write_read(x):
		arduino.write(bytes(x, 'utf-8'))
		time.sleep(0.05)
		# data = arduino.readline()
		# return data

while True:
	num = input("Enter the command in the form of 'number, meters, turns'")
	value = write_read(num)
	# print(value)

