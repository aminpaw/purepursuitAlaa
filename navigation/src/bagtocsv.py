#!/usr/bin/env python3
import rospy
import numpy as np
import csv
from nav_msgs.msg import Path

def add_point(data):
	global path_x,path_y,file
	writer = csv.writer(file)
	writer.writerow([data.poses[-1].pose.position.x,data.poses[-1].pose.position.y])
	rospy.loginfo("Received message")
		

rospy.init_node("listener")
path_sub = rospy.Subscriber("/odometry_node/trajectory",Path, add_point)
file = open('/home/arl/IPG_FSAI23/FS_autonomous11/ros/ros1_ws/src/fs_system/navigation/src/ipg_track.csv', 'w', newline="")

while not (rospy.is_shutdown() == True):
	pass
	
