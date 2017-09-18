#!/usr/bin/env python
import datetime
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64
from commander.srv import CommandStateResponse, CommandState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

import numpy as np
import sys
import copy
import threading

ARRIVAL_DISTANCE = 0.15
HOVER_HEIGHT = 1.0

FLIGHT_ENABLED = False

def q_to_yaw(q):
    return np.arctan2(2.0*(q.x*q.y + q.w*q.z), 1-2*(q.y*q.y + q.z*q.z))

class Controller:
    """
    Low level state control.  Simply controls velocity for moving to the next point.
    """
    def __init__(self):
        # ROS publishers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.play_pub = rospy.Publisher('/fc/path/play', Float64, queue_size=10)
        self.prop_start_pub = rospy.Publisher('/start_props', Empty, queue_size=10)
        self.prop_stop_pub = rospy.Publisher('/stop_props', Empty, queue_size=10)

        # ROS sucscribers
        rospy.Subscriber("/fc/cmd/pose", Twist, self.pathgen_pose)
        rospy.Subscriber("/pose", PoseStamped, self.pose_callback)

        self.base_pose = None
        self.goal_pose = np.array([0., 0., 0., 0.])
        self.path_poses = []
        self.cur_pose = np.array([0., 0., 0., 0.])
        self.Kp = np.array([.6, .6, .6, .8])
        # self.Kp = np.array([1., 1., 1., 1.])
        self.iter_loop = 0
        self.mode = "Idle"  # Idle, Takeoff, Land, Hover, Followpath
        self.logfile = open(str(datetime.datetime.now().isoformat()) + '.log', 'w+')

    def __del__(self):
        self.logfile.close()

    def pose_callback(self, msg):
        q = msg.pose.orientation
        yaw = q_to_yaw(q)

        # Need to set first base pose depending on where we start this program
        # TODO(heidt) if we have the global landmark matcher, we need to get rid
        # of this base_pose or a landmark match will wreck the system
        if self.base_pose is None:
            self.base_pose = np.array([msg.pose.position.x,
                                       msg.pose.position.y,
                                       msg.pose.position.z,
                                       yaw])
            self.log("base pose is".format(self.base_pose))
            
            return            

        measured_pose = np.array([msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z,
                                  yaw])
        self.cur_pose = measured_pose - self.base_pose

        # Proportional controller
        E = self.goal_pose - self.cur_pose

        # Bound the rotational error
        E[3] = (E[3] + np.pi) % 2*np.pi
        if E[3] < 0:
            E[3] += np.pi*2.
        E[3] -= np.pi

        y = E*self.Kp

        goal_vel = Twist()

        # rotate to global frame
        xy = np.array([[y[0]],[y[1]]])
        r = -self.cur_pose[3]

        xy_rot = np.array([[np.cos(r), -np.sin(r)],[np.sin(r), np.cos(r)]])
        res = xy_rot.dot(xy)

        goal_vel.linear.x = res[0]
        goal_vel.linear.y = res[1]
        goal_vel.linear.z = y[2]
        goal_vel.angular.z = y[3]

        self.iter_loop += 1
        if self.iter_loop % 150 == 0:
            self.log("Reading: {}".format(self.iter_loop // 60))
            # self.log(measured_pose)
            self.log("current pose: " + str(self.cur_pose))
            self.log("goal pose: " + str(self.goal_pose))

        y = np.clip(y, -.3, .3)
        y[3] = -y[3]

        if self.mode == "Takingoff":
            if FLIGHT_ENABLED:
                self.prop_start_pub.publish(Empty())
            goal_vel.linear.z = 0.4
            if abs(self.cur_pose[2] - HOVER_HEIGHT) < ARRIVAL_DISTANCE:
                self.mode = "Hovering"
                self.log("Reached hover height!")
        elif self.mode == "Landing":
            goal_vel.linear.z = -0.4
            if abs(self.cur_pose[2] - 0) < ARRIVAL_DISTANCE:
                self.mode = "Idle"
                self.log("Landed!")
                self.prop_stop_pub.publish(Empty())
        elif self.mode == "Followingpath":
            if np.linalg.norm(self.cur_pose - self.goal_pose) < ARRIVAL_DISTANCE:
                self.log("Reached point: {}".format(self.goal_pose))
                if len(self.path_poses) > 0:
                    self.goal_pose = self.path_poses.pop(0)
                else:
                    self.mode = "Hovering"
                    self.log("Done following path, hovering...")

        if self.mode != "Idle":
            self.pub.publish(goal_vel)

    def log(self, text):
        rospy.loginfo(text)
        self.logfile.write(str(text) + '\n')

    def set_state(self, command):
        #TODO(heidt) add a lock here!!!
        self.log("Attempting to set state to: {}".format(command))
        if command == "Takeoff":
            if self.mode == "Idle":
                self.goal_pose = np.array([0., 0., HOVER_HEIGHT, 0.])
                self.mode = "Takingoff"
        elif command == "Followpath":
            if self.mode in ["Hovering", "Followingpath"]:
                buildpath = rospy.ServiceProxy('fc/pathgen/build', Trigger)
                resp = buildpath()
                if resp.success == True:
                    # TODO(heidt) what is a good speed for this?
                    # TODO(heidt) should we continually publish this?
                    self.play_pub.publish(Float64(0.25))
                    self.mode = "Followingpath"
                else:
                    self.log("Bad response from pathgen!")

        elif command == "Hover":
            if self.mode in ["Followingpath"]:
                self.goal_pose = np.array([0., 0., HOVER_HEIGHT, 0.])
                self.mode = "Hovering"
        elif command == "Land":
            if self.mode in ["Followingpath", "Hovering"]:
                self.goal_pose = np.array([0., 0., 0., 0.])
                self.mode = "Landing"

    def pathgen_pose(self, msg):
        if self.mode == "Followingpath":
            lin = msg.linear
            pose = np.array([lin.x, lin.y, lin.z, msg.angular.z])
            self.goal_pose = pose

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
        if self.controller.mode == "Idle":
            if cmd == "Takeoff":
                self.controller.set_state(cmd)
            else:
                self._bad_state_log(self.controller.mode, cmd)
        elif self.controller.mode == "Hovering":
            if cmd in "Land":
                self.controller.set_state(cmd)
            if cmd == "Followpath":
                self.controller.set_state(cmd)
            else:
                self._bad_state_log(self.controller.mode, cmd)
        elif self.controller.mode == "Followingpath":
            if cmd in ["Hover", "Land"]:
                self.controller.set_state(cmd)
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