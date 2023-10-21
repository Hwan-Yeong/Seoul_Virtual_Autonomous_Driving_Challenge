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

        # rospy.Subscriber("/path", Path, self.path_callback)
        rospy.Subscriber("/custom_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/CollisionData", CollisionData, self.col_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.control_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_status = False
        self.is_col = False
        self.is_odom = False
        
        # PID
        # ==========================
        self.kp = 0.56
        self.ki = 0.0007
        self.kd = 0.2
        self.target_velocity = 30
        self.integral_limit = 10.0 # Clamping (anti_windup)
        # ==========================
        
        self.max_velocity = 50
        self.lookahead_distance = 0.0
        
        self.pid = PID(self.kp, self.ki, self.kd, self.target_velocity, self.integral_limit)
        # self.movav = MovingAverage(20)
        
        
        # TODO // IONIQ5 specifications
        # ===================================================
        self.vehicle_length = 4.635
        self.wheel_length = 3.0
        # ===================================================

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path == True and self.is_status == True:
                # self.current_waypoint = self.get_current_waypoint(self.status_msg, self.desired_path)

                # self.target_velocity 결정 (장애물 및 곡률 기반 속도 결정)

                # 종방향 속도제어 먼저         
                output_velocity = self.pid.compute(self.status_msg.velocity.x * 3.6) # [kph]
                
                if output_velocity > 0.0:
                    self.ctrl_cmd_msg.accel = 1.0
                    self.ctrl_cmd_msg.brake = 0.0
                    
                elif -5.0 < output_velocity <= 0.0:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 0.0
                    
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0

                # 속도 기반 lookahead_distance 만들어서 횡방향 조향 제어
                desired_path = self.path_to_array(self.lane_path)
                current_vel_magnitude = np.linalg.norm(np.array([self.status_msg.velocity.x, self.status_msg.velocity.y, self.status_msg.velocity.z]))
                self.lookahead_distance =  current_vel_magnitude / self.max_velocity * sqrt(desired_path[-1][0]**2 + desired_path[-1][1]**2)
                self.lookahead_point = min(desired_path, key=lambda c:abs(sqrt(c[0]**2 + c[1]**2)-self.lookahead_distance))


                if self.lookahead_point[0] != 0:
                    alpha = self.lookahead_point[1] / self.lookahead_point[0]
                    delta = atan2(2 * self.vehicle_length * alpha, self.lookahead_distance)
                else:
                    delta = 0 # go straight

                self.ctrl_cmd_msg.steering = delta
                
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

    def path_to_array(self, lane_path):
        if lane_path is None or len(lane_path.poses) != 60:
            return None

        desired_path = []

        for pose in lane_path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            desired_path.append((x, y))

        desired_path = np.array(desired_path)

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

class PID:
    def __init__(self, kp, ki, kd, target_value, integral_limit):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.target_value = target_value
        self.prev_error = 0
        self.integral = 0
        self.integral_limit = integral_limit

    def compute(self, current_value):
        
        error = self.target_value - current_value
        
        self.integral += error
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
            
        derivative = error - self.prev_error
        control_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        
        return control_output
    
if __name__ == '__main__':
    try:
        test_track = PurePursuit()

    except rospy.ROSInterruptException:
        pass
