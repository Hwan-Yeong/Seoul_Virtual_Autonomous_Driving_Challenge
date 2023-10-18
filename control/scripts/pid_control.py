#!/usr/bin/env python3

import time
import rospy
import numpy as np

from morai_msgs.msg  import EgoVehicleStatus, CollisionData, CtrlCmd
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
from math import sqrt

class Controller:
    def __init__(self):

        rospy.init_node('contorller', anonymous=True)
        rospy.Subscriber("/lane_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.control_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)

        self.is_path = False
        self.is_status = False

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.angle = 0.0
        self.pid = PID(0.56, 0.0007, 0.2)
        self.mm1 = MovingAverage(20)
        
        # TODO // Must be modified to IONIQ5 specifications
        # ===================================================
        self.vehicle_length = 4.355
        self.wheel_length = 3.5
        self.target_velocity = 30
        # ===================================================

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path == True and self.is_status == True:
                self.current_waypoint = self.get_current_waypoint(self.status, self.lane_path)

                steering = self.cal_stanley_control()
                self.ctrl_cmd_msg.steering = steering
                self.ctrl_cmd_msg.velocity = self.target_velocity
                # self.ctrl_cmd_msg.accel = 1.0
                # self.ctrl_cmd_msg.brake = 0.0


    def path_callback(self, msg):
        self.is_path = True
        self.lane_path = msg

    def status_callback(self, msg):
        self.is_status = True
        self.status = msg

    def get_current_waypoint(self, ego_status, lane_path):
        min_dist = float('inf')
        currnet_waypoint = -1
        for i, pose in enumerate(lane_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint
    
    def cal_stanley_control(self, ):
        steering = 0.1
        return steering

    
class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = [i for i in range(-1, n+1)]

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data.pop(0)
            self.data.append(new_sample)

    def get_mm(self):
        return sum(self.data) / len(self.data)

    def get_wmm(self):
        s = 0
        for i in range(-1, len(self.data)):
            s += self.data[i] * self.weights[i]
        return s / sum(self.weights[:len(self.data)])

class PID:
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte
        return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error
    
if __name__ == '__main__':
    try:
        test_track = Controller()

    except rospy.ROSInterruptException:
        pass