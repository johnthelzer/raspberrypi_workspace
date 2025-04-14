#!/usr/bin/env python3
import rospy
import serial
import numpy as np
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
port_name = '/dev/ttyACM0'
baudrate = 115200

roll = '0'
roll_old = '0'
roll_dot = '0'
roll_dot_old = '0'
pitch = '0'#
pitch_old = '0'
pitch_dot = '0'
pitch_dot_old = '0'
time_interval = 0.01



port = serial.Serial(port=port_name, baudrate=baudrate, timeout=0.25)
print('port opened')

rospy.init_node('serial_grabber', anonymous=True)
print('node started')
serial_pub = rospy.Publisher('/theta_info', Float32MultiArray, queue_size = 1)
print('topic published')
rate = rospy.Rate(100)
message_arrive_time_previous = rospy.Time.now().to_sec()
theta_array = Float32MultiArray()
input_string = String()
theta_array.data = []
print('serial_grabber starting now')

while not rospy.is_shutdown():
	if (port.inWaiting() > 0):
		incoming_data_string = port.read(port.inWaiting()).decode('utf-8')
		#incoming_data_string = port.readline().decode('utf-8')
		#debugging
		#print(incoming_data_string) #comment out when not needed
		if len(incoming_data_string) > 0:
			roll = 'r'
			pitch = 'p'
			roll_dot = 'q'
			pitch_dot = 'w'
			string_length = len(incoming_data_string)
            #find indices of delineators
			del_index_1 = 0
			del_index_2 = 1
			del_index_3 = 2
			del_index_4 = 3
			for i in range(len(incoming_data_string)):
				if incoming_data_string[i] == '!':
					del_index_1 = i
				elif incoming_data_string[i] == '@':
					del_index_2 = i
				elif incoming_data_string[i] == '#':
					del_index_3 = i
				elif incoming_data_string[i] == '$':
					del_index_4 = i
            #parse data, append non-delineators into strings
			for i in range(string_length):
				if(del_index_1 <= i < del_index_2):
					roll += incoming_data_string[i]
				elif(del_index_2 <= i < del_index_3):
					pitch += incoming_data_string[i]
				elif(del_index_3 <= i < del_index_4):
					roll_dot += incoming_data_string[i]
				elif(del_index_4 <= i):
					pitch_dot += incoming_data_string[i]
			#remove first character ('0') and check for blank strings
			if '!' not in roll:
				roll = roll_old
			else:
				roll = roll[2:]
			if '@' not in pitch:
				pitch = pitch_old
			else:
				pitch = pitch[2:]
			if '#' not in roll_dot:
				roll_dot = roll_dot_old
			else:
				roll_dot = roll_dot[2:]
			if '$' not in pitch_dot:
				pitch_dot = pitch_dot_old
			else:
				pitch_dot = pitch_dot[2:]
			if (roll == '') or (roll == '-'):
				roll = roll_old
			if (pitch == '') or (pitch == '-'):
				pitch = pitch_old
			if (roll_dot == '') or (roll_dot == '-'):
				roll_dot = roll_dot_old
			if (pitch_dot == '') or (pitch_dot == '-'):
				pitch_dot = pitch_dot_old
			#check indirectly for truncations by checking difference between old and new value
			try:
				roll_float = float(roll)
				if(abs(roll_float - float(roll_old)) > 1000):
					roll_float = float(roll_old)
				else:
					roll_old = roll
			except ValueError:
				roll_float = float(roll_old)
			try:
				pitch_float = float(pitch)
				if(abs(pitch_float - float(pitch_old)) > 1000):
					pitch_float = float(pitch_old)
				else:
					pitch_old = pitch
			except ValueError:
				pitch_float = float(pitch_old)
			try:
				roll_dot_float = float(roll_dot)
				if(abs(roll_dot_float - float(roll_dot_old)) > 1000):
					roll_dot_float = float(roll_dot_old)
				else:
					roll_dot_old = roll_dot
			except ValueError:
				roll_dot_float = float(roll_dot_old)
			try:
				pitch_dot_float = float(pitch_dot)
				if(abs(pitch_dot_float - float(pitch_dot_old)) > 1000):
					pitch_dot_float = float(pitch_dot_old)
				else:
					pitch_dot_old = pitch_dot
			except ValueError:
				pitch_dot_float = float(pitch_dot_old)
			#write data to messsage: roll, pitch, rolldot, pitchdot
			message_arrive_time = rospy.Time.now().to_sec()
			time_interval = message_arrive_time - message_arrive_time_previous
			theta_array.data = [roll_float, pitch_float, roll_dot_float, pitch_dot_float, time_interval]
			print(theta_array.data) ##comment this out when not needed
			serial_pub.publish(theta_array)
			message_arrive_time_previous = message_arrive_time
	rate.sleep()
