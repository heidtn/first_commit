#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

from commander.srv import CommandState

cur_pose = None

def show_help():
    print "Press t to takeoff\n" \
          "Press l to land\n" \
          "Press r to start record (ctrl+c to stop)\n" \
          "Press f to playback recorded path\n" \
          "Press z to get zero position\n" \
          "Press h for this menu\n"

def pose_callback(msg):
    global cur_pose
    cur_pose = msg

if __name__ == "__main__":
    global cur_pose
    rospy.init_node('commander_test', disable_signals=True, log_level=rospy.INFO)
    rospy.Subscriber("/pose", PoseStamped, pose_callback)

    show_help()
    rospy.loginfo("Waiting for service")
    rospy.wait_for_service('cmd_state')
    rospy.loginfo("Service found!")

    poselist = []
    zero_pose = None
    
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
                    pose_to_add = PoseStamped()
                    pose_to_add.pose.position.x = cur_pose.pose.position.x - zero_pose.pose.position.x
                    pose_to_add.pose.position.y = cur_pose.pose.position.y - zero_pose.pose.position.y
                    pose_to_add.pose.position.z = cur_pose.pose.position.z - zero_pose.pose.position.z

                    poselist.append(pose_to_add)
                    rospy.loginfo("Got pose: {}".format(pose_to_add))
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

        if command == "z":
            zero_pose = cur_pose
            rospy.loginfo("Got zero pose: {}".format(zero_pose))