#!/usr/bin/env python3

import roslib
import struct
import socket
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from nav_2d_msgs.msg import Polygon2DStamped, Polygon2D, Point2D
from time import sleep


# --- constants ---

HOST = '10.46.28.201'#'127.0.0.1'#'10.46.28.201'   # (local or external) address IP of remote server
PORT = 8000 # (local or external) port of remote server

pub = rospy.Publisher('/socket_receiver/detections',Polygon2DStamped,queue_size=1)
rospy.init_node('socky_rec')
r = rospy.Rate(20)

# --- create socket ---
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s = socket.socket() # default: socket.AF_INET, socket.SOCK_STREAM
print("SOCKET",s)
# --- connect to server ---
status = s.connect((HOST, PORT)) # one tuple (HOST, PORT), not two arguments
print("SOCKET START",status)
while not rospy.is_shutdown(): 
	print("IN")
	data = s.recv(1024).decode('ascii')
	data = data.split(',')
	print(data)
	polygonstp = Polygon2DStamped()

	found = True
	lop = []
	temp = []
	for i in range(len(data)):
		if float(data[i]) != -1.0:
			if i%2 == 0:
				temp.append(float(data[i]))
			else:
				temp.append(float(data[i]))
				if temp[0] >= 1.0:
					lop.append(Point2D(x=temp[0],y=temp[1]))
				temp = []
		else:
			found = False
			break

	if found:
		polygonstp.polygon = Polygon2D(lop)
	else:
		polygonstp.polygon = Polygon2D([])
	pub.publish(polygonstp)
	r.sleep()
	print(rospy.is_shutdown())

	
s.close()
print("TERMINATE")
