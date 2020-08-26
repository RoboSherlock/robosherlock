#!/usr/bin/env python
"""
#@Author:   Frankln Kenghagho
#@Date:     25.08.2020
#@Project:  RobotVA
"""



#This program is frontend
#   1- Remotely launch RobotVQA

#setting python paths
import socket
import sys
import rospy

class RobotVQA(object):

	def __init__(self):
		rospy.init_node('robotvqa_launcher')
		rospy.loginfo('Starting Ros Node RobotVQA Launcher ...')
		rospy.on_shutdown(self.cleanup)  
                HOST=rospy.get_param('ip_address','192.168.102.3')
		PORT = rospy.get_param('port','9000')
		data = "RobotVQA"
		# Create a socket (SOCK_STREAM means a TCP socket)
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
		    # Connect to server and send data
		    print "Connecting to ",(HOST, int(PORT))
		    sock.connect((HOST, int(PORT)))
                    print "Connected to ",(HOST, int(PORT))
		    sock.sendall(data + "\n")
		    # Receive data from the server and shut down
		    received = sock.recv(1024)
		finally:
		    sock.close()

	def cleanup(self):
		rospy.logwarn('Shutting down RobotVQA launcher ...')
		
# main loop
if __name__=="__main__":
    
    try:
        #start the launcher
	rvqa=RobotVQA()
    except Exception as e:
	rospy.logwarn('Shutting down RobotVQA launcher ...'+str(e))
