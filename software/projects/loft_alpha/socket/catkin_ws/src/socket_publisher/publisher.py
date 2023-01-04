#!/usr/bin/env python3

import roslib
import socket
import rospy
from std_msgs.msg import Float32MultiArray
from nav_2d_msgs.msg import Polygon2DStamped, Polygon2D, Point2D
from copy import deepcopy
import struct
import sys
import threading
import time
import keyboard


# --- constants ---

HOST = '127.0.0.1'   # local address IP (not external address IP)

				# '0.0.0.0' or '' - conection on all NICs (Network Interface Card),
				# '127.0.0.1' or 'localhost' - local conection only (can't connect from remote computer)
				# 'Local_IP' - connection only on one NIC which has this IP

PORT = 8000 # local port (not external port)

data_msg = ''
pub_state  = True
status = True

def callback(data):
	global data_msg, pub_state
	
	combinedstr = ''
	if len(data.polygon.points) == 0:
		combinedstr = '-1,-1'
	else:
		for i in range(len(data.polygon.points)):
			combinedstr += str(data.polygon.points[i].x)
			combinedstr += ","
			combinedstr += str(data.polygon.points[i].y)
			if i != (len(data.polygon.points)-1):
				combinedstr += ","
	data_msg = deepcopy(combinedstr)
	pub_state = True


def handle_client(conn, addr):
	global pub_state, status
	print(status)
	while status and (not rospy.is_shutdown()):
		if pub_state:
			conn.send(data_msg.encode('ascii'))
			pub_state = False									
  

s = socket.socket() 
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

s.bind((HOST, PORT)) # one tuple (HOST, PORT), not two arguments

s.listen(1) # number of clients waiting in queue for "accept".
			# If queue is full then client can't connect.

pub = rospy.Subscriber('/detections',Polygon2DStamped,callback)
rospy.init_node('socky_pub')    
while not rospy.is_shutdown():
	# --- accept client ---
	conn, addr = s.accept() 
	t = threading.Thread(target=handle_client, args=(conn, addr))
	t.start()

	if keyboard.is_press('space'):
		conn.close()
		status = False
		s.close()
		break

