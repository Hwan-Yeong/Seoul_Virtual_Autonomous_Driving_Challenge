#!/usr/bin/env python3

import time
import rospy
import numpy as np

from tf.transformations import euler_from_quaternion
from math import cos, sin, pi, sqrt, pow, atan2

from morai_msgs.msg  import EgoVehicleStatus, CollisionData, CtrlCmd
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry


class PurePursuit:
    def __init__(self):

        rospy.init_node('pure_pursuit', anonymous=True)

        rospy.Subscriber("/path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/CollisionData", CollisionData, self.col_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.control_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path = False
        self.is_status = False
        self.is_col = False
        self.is_odom = False

        self.angle = 0.0
        self.lookahead_distance = 0.0
        # self.pid = PID(0.56, 0.0007, 0.2)
        # self.mm1 = MovingAverage(20)
        
        # TODO // IONIQ5 specifications
        # ===================================================
        self.vehicle_length = 4.635
        self.wheel_length = 3.0
        self.target_velocity = 30
        self.max_velocity = 50
        # ===================================================

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path == True and self.is_status == True and self.is_odom == True:
                # self.current_waypoint = self.get_current_waypoint(self.status_msg, self.desired_path)

                # 종방향 속도제어 먼저 하고,

                # 속도 기반 lfd 만들어거 횡방향 조향 제어
                desired_path = self.coordinate_transform_global2vehicle(self.lane_path)
                self.lookahead_distance = self.status_msg.velocity / self.max_velocity * sqrt(desired_path[-1][0]**2 + desired_path[-1][1]**2)
                self.lookahead_point = min(desired_path, key=lambda c:abs(sqrt(c[0]**2 + c[1]**2)-self.lookahead_distance))

                alpha = self.lookahead_point[1]/self.lookahead_point[0]
                delta = atan2(2*self.vehicle_length*alpha, self.lookahead_distance)

                self.ctrl_cmd_msg.velocity = self.target_velocity
                self.ctrl_cmd_msg.steering = delta
                # self.ctrl_cmd_msg.accel = 1.0
                # self.ctrl_cmd_msg.brake = 0.0
                
                self.control_pub.publish(self.ctrl_cmd_msg)
                
            rate.sleep()


    def path_callback(self, msg):
        self.is_path = True
        self.lane_path = msg

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

    def col_callback(self, msg):
        self.is_col = True
        self.col_msg = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    # def get_current_waypoint(self, ego_status, desired_path):
    #     min_dist = float('inf')
    #     currnet_waypoint = -1
    #     for i, pose in enumerate(desired_path.poses):
    #         dx = ego_status.position.x - pose.pose.position.x
    #         dy = ego_status.position.y - pose.pose.position.y

    #         dist = sqrt(pow(dx, 2) + pow(dy, 2))
    #         if min_dist > dist:
    #             min_dist = dist
    #             currnet_waypoint = i
    #     return currnet_waypoint

    def coordinate_transform_global2vehicle(self, lane_path):
        x = lane_path.pose.pose.position.x
        y = lane_path.pose.pose.position.y
        # =================================================================
        # 기존 path가 취하는 좌표계 => 차량 좌표계로 변환시켜주는 과정 필요
        # lane_path에서 어떤 기준 좌표계를 사용하는지 몰라 구체적인 건 아직 못짬
        # =================================================================
        # desired_path에 담기는 정보가 [(x0, y0), (x1, y1), ... , (xf, yf)] 형태가 되어야 한다.
        desired_path = lane_path
        return desired_path
    
    def cal_stanley_control(self, ):
        steering = 0.0
        return steering

    
# class MovingAverage:
#     def __init__(self, n):
#         self.samples = n
#         self.data = []
#         self.weights = [i for i in range(-1, n+1)]

#     def add_sample(self, new_sample):
#         if len(self.data) < self.samples:
#             self.data.append(new_sample)
#         else:
#             self.data.pop(0)
#             self.data.append(new_sample)

#     def get_mm(self):
#         return sum(self.data) / len(self.data)

#     def get_wmm(self):
#         s = 0
#         for i in range(-1, len(self.data)):
#             s += self.data[i] * self.weights[i]
#         return s / sum(self.weights[:len(self.data)])

# class PID:
#     def __init__(self, kp, ki, kd):
#         self.Kp = kp
#         self.Ki = ki
#         self.Kd = kd
#         self.p_error = 0.0
#         self.i_error = 0.0
#         self.d_error = 0.0

#     def pid_control(self, cte):
#         self.d_error = cte - self.p_error
#         self.p_error = cte
#         self.i_error += cte
#         return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error
    
if __name__ == '__main__':
    try:
        test_track = PurePursuit()

    except rospy.ROSInterruptException:
        pass
