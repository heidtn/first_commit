#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from commander.srv import CommandStateResponse, CommandState
from std_msgs.msg import Empty

import numpy as np
import sys
import copy
import threading

ARRIVAL_DISTANCE = 0.07
HOVER_HEIGHT = 1.0


def q_to_yaw(q):
    return np.arctan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)

class Controller:
    """
    Low level state control.  Simply controls velocity for moving to the next point.
    """
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.prop_start_pub = rospy.Publisher('/start_props', Empty, queue_size=10)
        self.prop_stop_pub = rospy.Publisher('/stop_props', Empty, queue_size=10)
        rospy.Subscriber("/pose", PoseStamped, self.pose_callback)

        self.base_pose = None
        self.goal_pose = np.array([0., 0., 0., 0.])
        self.path_poses = []
        self.cur_pose = np.array([0., 0., 0., 0.])
        self.Kp = np.array([.6, .6, .6, 6.])
        self.iter_loop = 0
        self.mode = "Idle"  # Idle, Takeoff, Land, Hover, Followpath

    def pose_callback(self, msg):
        q = msg.pose.orientation
        yaw = q_to_yaw(q)
        if self.base_pose is None:
            self.base_pose = np.array([msg.pose.position.x,
                                       msg.pose.position.y,
                                       msg.pose.position.z,
                                       yaw])
            rospy.loginfo("base pose is".format(self.base_pose))
            return            

        self.cur_pose = np.array([msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z,
                                  yaw]) - self.base_pose

        # Proportional controller
        E = self.goal_pose - self.cur_pose
        y = E*self.Kp

        y = np.clip(y, -.3, .3)
        y[3] = -y[3]

        goal_vel = Twist()

        self.iter_loop += 1
        if self.iter_loop % 60 == 0:
            rospy.loginfo("Reading: {}".format(self.iter_loop % 60))
            rospy.loginfo(E)
            rospy.loginfo(self.cur_pose)
            rospy.loginfo(self.goal_pose)


        # rotate to global frame
        xy = np.array([[y[0]],[y[1]]])
        r = -self.cur_pose[3]

        xy_rot = np.array([[np.cos(r), -np.sin(r)],[np.sin(r), np.cos(r)]])
        res = xy_rot.dot(xy)

        goal_vel.linear.x = res[0]
        goal_vel.linear.y = res[1]
        goal_vel.linear.z = y[2]
        goal_vel.angular.z = y[3]

        if self.mode == "Takingoff":
            self.prop_start_pub.publish(Empty())
            goal_vel.linear.z = 0.4
            if abs(self.cur_pose[2] - HOVER_HEIGHT) < ARRIVAL_DISTANCE:
                self.mode = "Hovering"
                rospy.loginfo("Reached hover height!")
        elif self.mode == "Landing":
            goal_vel.linear.z = -0.4
            if abs(self.cur_pose[2] - 0) < ARRIVAL_DISTANCE:
                self.mode = "Idle"
                rospy.loginfo("Landed!")
                self.prop_stop_pub.publish(Empty())
        elif self.mode == "Followingpath":
            if abs(self.cur_pose - self.goal_pose) < ARRIVAL_DISTANCE:
                rospy.loginfo("Reached point: {}".format(self.goal_pose))
                if len(self.path_poses) > 0:
                    self.goal_pose = self.path_poses.pop(0)
                else:
                    self.mode = "Hovering"
                    rospy.loginfo("Done following path, hovering...")

        if self.mode != "Idle":
            self.pub.publish(goal_vel)

    def set_state(self, command, path=None):
        #TODO(heidt) add a lock here!!!
        rospy.loginfo("Attempting to set state to: {}".format(command))
        if command == "Takeoff":
            if self.mode == "Idle":
                self.goal_pose = np.array([0., 0., HOVER_HEIGHT, 0.])
                self.mode = "Takingoff"
        elif command == "Followpath":
            if self.mode in ["Hovering", "Followingpath"]:
                self.path_poses = []
                for pose in path:
                    yaw = q_to_yaw(pose.orientation)
                    p = np.array([pose[0], pose[1], pose[2], yaw])
                    self.path_poses.append(p)
                    self.mode = "Followingpath"
        elif command == "Hover":
            if self.mode in ["Followingpath"]:
                self.goal_pose = np.array([0., 0., HOVER_HEIGHT, 0.])
                self.mode = "Hovering"
        elif command == "Land":
            if self.mode in ["Followingpath", "Hovering"]:
                self.goal_pose = np.array([0., 0., 0., 0.])
                self.mode = "Landing"

    def set_goal_pose(self, pose):
        self.goal_pose = np.array(pose)

    def get_current_pose(self):
        return self.cur_pose

    def distance_to_goal(self):
        return np.linalg.norm(self.cur_pose - self.goal_pose)



class Commander:
    """
    Handles high level state transition commands.
    """
    def __init__(self):
        self.pub = rospy.Publisher('/state', String, queue_size=10)
        #rospy.Subscriber("/cmd_state", String, self.cmd_state)
        rospy.Service('/cmd_state', CommandState, self.cmd_state)
        self.command_state = "Idle" # Takeoff, Land, Followpath, Hover
        rospy.loginfo("Current state is Idle")
        self.controller = Controller()

        command_loop_thread = threading.Thread(target=self.command_loop)
        command_loop_thread.daemon = True
        command_loop_thread.start()

    def command_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pub.publish(self.controller.mode)
            rate.sleep()

    def cmd_state(self, msg):
        cmd = msg.cmd
        poses = msg.poses
        if self.controller.mode == "Idle":
            if cmd == "Takeoff":
                self.controller.set_state(cmd)
            else:
                self._bad_state_log(self.controller.mode, cmd)
        elif self.controller.mode == "Hovering":
            if cmd in ["Land", "Followpath"]:
                self.controller.set_state(cmd)
            else:
                self._bad_state_log(self.controller.mode, cmd)
        elif self.controller.mode == "Followingpath":
            if cmd in ["Hover", "Land"]:
                self.controller.set_state(cmd, path=poses)
            else:
                self._bad_state_log(self.controller.mode, cmd)
        else:
            self._bad_state_log(self.controller.mode, cmd)

        resp = CommandStateResponse()
        resp.success = True  # TODO(heidt) this should be false if a bad transition
        resp.state = self.controller.mode
        return resp

    def _bad_state_log(self, cur, cmd):
        rospy.logwarn("Received bad state transition: {} to {}".format(cur, cmd))


def shutdown():
    sys.exit()

if __name__ == "__main__":
    rospy.init_node('fc_commander', disable_signals=True, log_level=rospy.INFO)
    rospy.loginfo("Spinning up commander!")
    c = Commander()
    rospy.spin()