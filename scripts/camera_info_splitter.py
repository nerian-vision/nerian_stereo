#!/usr/bin/env python

# This script splits an incoming StereoCameraInfo message into two
# CameraInfo messages, making it easier to integrate the nerian_stereo
# node with existing nodes that process 2d camera data.

import rospy
from nerian_stereo.msg import StereoCameraInfo
from sensor_msgs.msg import CameraInfo

leftPub = 0
rightPub = 0

def callback(data):
    global leftPub;
    global rightPub;

    rospy.loginfo(rospy.get_caller_id() + ": Camera info received")
    leftPub.publish(data.left_info)
    rightPub.publish(data.right_info)

def listener():
    global leftPub;
    global rightPub;

    rospy.init_node('camera_info_splitter')
    rospy.Subscriber("/nerian_stereo/stereo_camera_info", StereoCameraInfo, callback)

    leftPub = rospy.Publisher('/nerian_stereo/left_camera_info', CameraInfo, queue_size=10)
    rightPub = rospy.Publisher('/nerian_stereo/right_camera_info', CameraInfo, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
