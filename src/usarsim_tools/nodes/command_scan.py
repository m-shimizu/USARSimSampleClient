#!/usr/bin/env python
import roslib
roslib.load_manifest('usarsim_tools')
import rospy
from usarsim_inf.msg import RangeImageScan

master = rospy.client.get_master()
statusTopics = filter(lambda x: x[1] == "usarsim_inf/RangeImageScan", master.getTopicTypes()[2])
publishers = []
for topic in statusTopics:
	publishers.append(rospy.Publisher("KinectDepth/command", RangeImageScan))

rospy.init_node("kinect_scan_command")
command = RangeImageScan()
command.header.stamp = rospy.Time.now()

rospy.sleep(0.5)
for pub in publishers:
	pub.publish(command)
