#!/usr/bin/env python

## Philippe Sayegh
## 260637812


import rospy
import tf
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys

from sensor_msgs.msg import LaserScan

from dynamic_reconfigure.server import Server
from wall_following_assignment.cfg import pidTweakConfig


class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
        
    def update_control(self, current_error, reset_prev=False):
        
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.sum_error = self.sum_error + current_error
        
        self.prev_error_deriv = self.curr_error_deriv
        self.curr_error_deriv = (self.curr_error - self.prev_error)/self.dt

        self.control = self.Kp * (self.curr_error + 1/self.Ti * self.sum_error + self.Td * self.curr_error_deriv)

    
        
        
    def get_control(self):
        return self.control
        
class WallFollowerHusky:
    def __init__(self):
        rospy.init_node('wall_follower_husky', anonymous=True)

        self.forward_speed = rospy.get_param("~forward_speed")
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall")
        self.hz = 50

        self.controller = PID(4, 1, 1000, 0.02)
        

        self.server = Server(pidTweakConfig, self.config_callback)

        self.cte_pub = rospy.Publisher('husky_1/cte', String, queue_size=10)

        # todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
        # using geometry_msgs.Twist messages
        self.cmd_pub = rospy.Publisher('husky_1/cmd_vel', Twist, queue_size=10)

    
        # todo: set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spinOnce() call, as long as a laser scan
        # message has been published in the meantime by another node
        self.laser_sub = rospy.Subscriber('/husky_1/scan', LaserScan, self.laser_scan_callback) 

        

    def config_callback(self, config, level):

        self.controller.Kp = config['Kp']
        self.controller.Td = config['Td']
        self.controller.Ti = config['Ti']

        return config



        
    def get_min_range(self, msg):
        a_min = 100000000
        ranges = msg.ranges
        for i in range(len(msg.ranges) - len(ranges)/2):
            if ranges[i] < a_min:
                a_min = ranges[i]
        return a_min

        
    def laser_scan_callback(self, msg):

        min_range = self.get_min_range(msg)

        error = min_range - self.desired_distance_from_wall


        self.cte_pub.publish(str(error))
        

        self.controller.update_control(error)


        cmd = Twist()

        
        cmd.angular.z = (self.controller.get_control())
        
        cmd.linear.x = self.forward_speed

        if (msg.ranges[len(msg.ranges)/2 - 1] < 2.1) and\
           (msg.ranges[602] > 3 or msg.ranges[116] > 3) and\
           (msg.ranges[652] > 3 or msg.ranges[61] > 3) and\
           (msg.ranges[518] < 3.75 and\
            msg.ranges[182] < 3.75):
            cmd.angular.z = -1.7

        self.cmd_pub.publish(cmd)
        
        
        

        
   
            
    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            rate.sleep()

    
if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()


