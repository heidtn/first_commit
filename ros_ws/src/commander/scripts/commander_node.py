#!/usr/bin/env python
import datetime
import time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64
from commander.srv import CommandStateResponse, CommandState
from pathgen.msg import PathGenStatus
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

import numpy as np
import sys
import copy
import threading

LAND_THRESH = 0.5
LAND_SPEED_THRESH = 0.05

ARRIVAL_DISTANCE = 0.15
HOVER_HEIGHT = 1.76
PLAY_SPEED = 0.5

FLIGHT_ENABLED = True

USE_TWIST = False
USE_BIAS = True

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
        if USE_BIAS:
            rospy.Subscriber("/fc/cmd/visaligned_pose", Twist, self.handle_bias)
        rospy.Subscriber("/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/fc/path/status", PathGenStatus, self.pathgen_status)

        self.base_pose = None
        self.goal_pose = np.array([0., 0., 0., 0.])
        self.path_poses = []
        self.cur_pose = np.array([0., 0., 0., 0.])

        # Ziegler nichols, assume 1 for Ku, 2 for Tu
        self.Kp = np.array([1.2, 1.2, 1.2, 1.8])
        self.Ki = np.array([.6, .6, .6, .8])
        self.Kd = np.array([.05, .05, .05, .1])
        self.previous_error = 0
        self.last_time = time.time()
        self.integration = np.array([0, 0, 0, 0])
        self.integration_max = .5
        
        self.bias = np.array([0., 0., 0.])

        self.endtime = 99999999
        self.currenttime = 0
	self.is_path_loaded = False
        self.iter_loop = 0
        self.mode = "Idle"  # Idle, Takeoff, Land, Hover, Followpath
        #self.logfile = open(str(datetime.datetime.now().isoformat()) + '.log', 'w+')

    def __del__(self):
        pass #self.logfile.close()

    def handle_bias(self, msg):
        self.bias = np.array([msg.linear.x,
                              msg.linear.y,
                              msg.linear.z,
                              0])

    def pose_callback(self, msg):
        if msg.header.frame_id != "/odom":
            return
        
        if not USE_TWIST:
            q = msg.pose.orientation
            yaw = q_to_yaw(q)

            measured_pose = np.array([msg.pose.position.x,
                                      msg.pose.position.y,
                                      msg.pose.position.z,
                                      yaw])
        else:
           measure_pose = np.array([msg.linear.x,
                                    msg.linear.y,
                                    msg.linear.z,
                                    msg.angular.z])

        self.cur_pose = measured_pose
	self.cur_pose = self.cur_pose - self.bias

        # Proportional controller
        E = self.goal_pose - self.cur_pose

        # Bound the rotational error
        E[3] = np.mod((E[3] + np.pi), 2.*np.pi)
        if E[3] < 0:
            E[3] += np.pi*2.
        E[3] -= np.pi

        # PID calculation
        current_time = time.time()
        delta_t = current_time - self.last_time
        self.last_time = current_time

        self.integration += E*delta_t
        self.integration = np.clip(self.integration, -self.integration_max, self.integration_max)
        d = (E - self.previous_error) / delta_t
        self.previous_error = E

        y = E*self.Kp + d*self.Kd

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
            goal_vel.linear.z = 0.6
            if self.cur_pose[2] > HOVER_HEIGHT:
                self.mode = "Hovering"
                self.log("Reached hover height!")
        elif self.mode == "Landing":
            goal_vel.linear.z = -0.2
            if (self.cur_pose[2] < LAND_THRESH and d[2] < LAND_SPEED_THRESH):
                self.mode = "Idle"
                self.log("Landed!")
                self.prop_stop_pub.publish(Empty())
        elif self.mode == "Followingpath":
            if np.linalg.norm(self.cur_pose - self.goal_pose) < ARRIVAL_DISTANCE:
                self.log("Reached point: {}".format(self.goal_pose))
                if self.currenttime == self.endtime and self.is_path_loaded:
                    self.mode = "Hovering"
                    self.log("Done following path, hovering...")
        elif self.mode == "Kill":
            self.prop_stop_pub.publish(Empty())
            self.mode = "Idle"

        if self.mode != "Idle":
            self.pub.publish(goal_vel)

    def log(self, text):
        rospy.loginfo(text)
        #self.logfile.write(str(text) + '\n')

    def set_state(self, command):
        #TODO(heidt) add a lock here!!!
        self.log("Attempting to set state to: {}".format(command))
        if command == "Takeoff":
            if self.mode == "Idle":
                self.goal_pose = np.array([self.cur_pose[0], self.cur_pose[1], HOVER_HEIGHT, 0.])
                self.mode = "Takingoff"
        elif command == "Followpath":
            if self.mode in ["Hovering", "Followingpath"]:
                buildpath = rospy.ServiceProxy('fc/path/build', Trigger)
                resp = buildpath()
                if resp.success == True:
                    self.log("Sucessfully built path")
                    # TODO(heidt) what is a good speed for this?
                    # TODO(heidt) should we continually publish this?
                    self.play_pub.publish(Float64(PLAY_SPEED))
                    self.mode = "Followingpath"
                else:
                    self.log("Bad response from pathgen!")

        elif command == "Hover":
            if self.mode in ["Followingpath"]:
                self.goal_pose = np.array([self.cur_pose[0], self.cur_pose[1], HOVER_HEIGHT, 0.])
                self.mode = "Hovering"
        elif command == "Land":
            if self.mode in ["Followingpath", "Hovering"]:
                self.goal_pose = np.array([self.cur_pose[0], self.cur_pose[1], 0., 0.])
                self.mode = "Landing"
        elif command == "Kill":
            self.mode = "Kill"

    def pathgen_pose(self, msg):
        if self.mode == "Followingpath":
            lin = msg.linear
            pose = np.array([lin.x, lin.y, lin.z, msg.angular.z])
            self.goal_pose = pose

    def pathgen_status(self, msg):
        self.endtime = msg.endtime
        self.currenttime = msg.currenttime
        self.is_path_loaded = msg.pathloaded

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
            if cmd in ["Takeoff", "Followpath"]:
                self.controller.set_state(cmd)
            else:
                self._bad_state_log(self.controller.mode, cmd)
        elif self.controller.mode == "Hovering":
            if cmd in ["Land", "Kill", "Followpath"]:
                self.controller.set_state(cmd)
            else:
                self._bad_state_log(self.controller.mode, cmd)
        elif self.controller.mode == "Followingpath":
            if cmd in ["Hover", "Land", "Kill"]:
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
