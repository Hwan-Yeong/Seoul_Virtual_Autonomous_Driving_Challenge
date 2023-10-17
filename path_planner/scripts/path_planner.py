#!/usr/bin/env python3

import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus, CollisionData
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np

class PathPlanner:
    def __init__ (self):
        rospy.init_node("path_planner", anonymous=True)
        
        rospy.Subscriber("/lane_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber("CollisionData", CollisionData, self.col_callback)
        
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        
        self.is_path = False
        self.is_ego = False
        self.is_col = False
        
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_ego and self.is_col:
                
        
    def path_callback(self, msg):
        self.is_path = True
        self.lattice_path = msg
        
    def ego_callback(self, msg):
        self.is_ego = True
        self.egot_msg = msg
        
    def col_callback(self, msg):
        self.is_col = True
        self.col_msg = msg