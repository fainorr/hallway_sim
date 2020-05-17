#!/usr/bin/env python

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from sensor_msgs.msg import *
from numpy import *
import os
import time
import sys
import lidar_compare
from quad_analysis_methods import *

# -----------
# EXPLORE FSM
# -----------

class explore_FSM():

    def __init__(self):

        self.dT = 0.1
        self.timenow = time.time()
        self.oldtime = self.timenow

        # --- navigation parameters ---
        self.obst_size = 3;         # number of consecutive dots
        self.safe_range = 2.0;         # search ranges for obstacles

        # initialize
        self.distances = zeros(360)
        self.angle_parameters = [0.0, 0.0, 0.0]

        self.old_action = "stand"
        self.old_direction = "left"
        self.arm_action = 'rest'

        self.analyze = lidar_compare.lidar_compare()

        # subscribe to rplidar and explore action
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.scancallback)
        self.FSM_action = rospy.Subscriber('/action', String, self.actioncallback)
        self.FSM_direction = rospy.Subscriber('/direction', String, self.directioncallback)

        # publish action and direction for the robot controller
        self.FSM_action = rospy.Publisher('/action', String, queue_size=1)
        self.FSM_direction = rospy.Publisher('/direction', String, queue_size=1)
        self.arm_action_pub = rospy.Publisher('/arm_action', String, queue_size=1)

        # create loop
        rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

    def loop(self, event):

        self.timenow = time.time()
        self.oldtime = self.timenow

        self.explore_action = "wander"

        self.r_pos = zeros(len(self.distances))
        self.in_range = zeros(len(self.distances))

        self.r_pos[0:45] = self.distances[315:360]
        self.r_pos[45:360] = self.distances[0:315]

        for i in range(0,len(self.distances)):
            if self.distances[i] > self.safe_range: self.in_range[i] = 0
            else: self.in_range[i] = 1

        # --- ANALYZE SCAN ---

        self.quad_obstacles =[0.,0.,0.,0.]
        self.quad_obstacles = analyze_quadrant(self.obst_size, self.in_range)

        if self.quad_obstacles[2] == 1:
            self.explore_action = "press"

        if self.explore_action == "wander":

            # find action and direction from the lidar_compare.py script
            self.command_history = [self.old_action, self.old_direction]
            self.safe_range = 0.8
            self.action, self.direction = self.analyze.find_optimal_action(self.distances, self.angle_parameters, self.obst_size, self.safe_range, self.command_history)
            self.arm_action = "rest"

        elif self.explore_action == "press":
            self.action = "stand"
            self.direction = "left"
            self.arm_action = "press"

        # store results in string messages and publish
        self.action_msg = String()
        self.action_msg.data = self.action

        self.direction_msg = String()
        self.direction_msg.data = self.direction

        self.FSM_action.publish(self.action_msg)
        self.FSM_direction.publish(self.direction_msg)

        self.arm_action_pub.publish(self.arm_action)


    def scancallback(self,data):

        self.distances = array(data.ranges)
        self.angle_parameters = [data.angle_min, data.angle_max, data.angle_increment]

    def actioncallback(self,data):

        self.old_action = data.data

    def directioncallback(self,data):

        self.old_direction = data.data


# main function

def main(args):
    rospy.init_node('explore_FSM', anonymous=True)
    myNode = explore_FSM()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
