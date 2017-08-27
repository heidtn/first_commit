#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from commander.srv import CommandState

cur_pose = None

def show_help():
    print "Press t to takeoff\n" \
          "Press l to land\n" \
          "Press r to start record (ctrl+c to stop)\n" \
          "Press p to playback recorded path\n" \
          "Press h for this menu\n"

def pose_callback(msg):
    global cur_pose

if __name__ == "__main__":
    global cur_pose
    rospy.init_node('commander_test', disable_signals=True, log_level=rospy.INFO)
    rospy.Subscriber("/pose", PoseStamped, pose_callback)

    show_help()
    rospy.loginfo("Waiting for service")
    rospy.wait_for_service('cmd_state')
    rospy.loginfo("Service found!")

    poselist = []
    
    while True:
        command = raw_input("Enter number for command (h for help): ")
        print command
        if command == "l":
            try:
                land = rospy.ServiceProxy('cmd_state', CommandState)
                resp = land("Land", None)
                rospy.loginfo("Completed land request with response: {}".format(resp))
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        if command == "t":
            try:
                takeoff = rospy.ServiceProxy('cmd_state', CommandState)
                resp = takeoff("Takeoff", None)
                rospy.loginfo("Completed Takeoff request with response: {}".format(resp))
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        if command == "h":
            show_help()
        if command == "r":
            poselist = []
            rate = rospy.Rate(1)
            try:
                while True:
                    poselist.append(cur_pose)
                    rate.sleep()
            except KeyboardInterrupt:
                rospy.loginfo("Got {} poses".format(len(poselist)))
        if command == "f":
            try:
                takeoff = rospy.ServiceProxy('cmd_state', CommandState)
                resp = takeoff("Followpath", poselist)
                rospy.loginfo("Completed Takeoff request with response: {}".format(resp))
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e