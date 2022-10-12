#!/usr/bin/env  python
import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist
from lust_msgs.msg import lust_cmd
import socket
import struct
import numpy

global boolStart

global timeStamp
global sock

#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "192.168.65.219"#"192.254.82.154"
UDP_PORT = 4295

timeStamp = 0.05

def callback(msg):

	global timeStamp
	frame = bytearray()
	#rospy.loginfo(data)
	global sock
	#rospy.loginfo(rospy.get_caller_id()  + "I heard %s", data.data)
	#rospy.loginfo(rospy.get_caller_id() + "test: " + str(data.x))
	
	
	
	frame.append(ord('S'))
	frame.append(ord('S'))

	# IO
#	frame.append(ord('8'))
	BoolSend = [msg.start_run, msg.stop_run, False, False, False, False, False, False]
	#print(msg.start_run)
	#print(msg.stop_run)
	#print(BoolSend)

	twist = msg.cmd_vel
	ba = bytearray(struct.pack("f", twist.linear.x))
	for item in ba:
		frame.append(item)
	ba = bytearray(struct.pack("f", twist.angular.z))
	for item in ba:
		frame.append(item)
	ba = bytearray(struct.pack("f", twist.angular.x))
	for item in ba:
		frame.append(item)
	ba = bytearray(struct.pack("f", timeStamp))
	for item in ba:
		frame.append(item)
	item = struct.pack("8?", *BoolSend)
	frame.append(item[0])
	#item1 = struct.pack("8?", *BoolSend2)
	frame.append(item[1])
	
	#frame.append(item[1])
	#frame.append(item[2])
	timeStamp = timeStamp + 0.05
	#frame.append(ord('T'))
	#frame.append(ord('T'))

	rospy.loginfo(str(twist.linear.x))
	rospy.loginfo(str(twist.angular.z))
	rospy.loginfo(str(twist.angular.x))
	rospy.loginfo(str(frame))
	sock.sendto(frame, (UDP_IP, UDP_PORT))
	#rate.sleep



def listener():
	global sock
	rospy.loginfo("v listener funkciji")
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	rospy.init_node('prenos', anonymous=True)
	rospy.Subscriber("nanoscan3_safety", lust_cmd, callback)
	rospy.Subscriber("/cmd_vel", lust_cmd, callback)
	
	rospy.spin()

if __name__ == '__main__':
	rospy.loginfo("v __main__ funkciji: ")
	listener()
