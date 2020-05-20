#!/usr/bin/env python
#! -*- coding: utf-8 -*-
""" task_setup.py - Version 1.0 2013-12-20

    Set up a number of waypoints and a charging station for use with simulated tasks using
    SMACH and teer.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import  pi
from collections import OrderedDict

def setup_task_environment(self):
    # 设置机器人巡逻的区域边长（正方形）
    self.square_size = rospy.get_param("~square_size", 1.0) # meters
    
    # 设置低电量的阈值(between 0 and 100)
    self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 50)
    
    # 设置巡逻的次数
    self.n_patrols = rospy.get_param("~n_patrols", 2)
    
    # 每个waypoint超时时间
    self.move_base_timeout = rospy.get_param("~move_base_timeout", 10) #seconds
    
    # 初始化巡逻计数器
    self.patrol_count = 0
    
    # 订阅move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    
    # action server超时时间为60s
    self.move_base.wait_for_server(rospy.Duration(60))    
    
    rospy.loginfo("Connected to move_base action server")
    
    # 创建列表保存目标的四元数(位姿)
    quaternions = list()
    
    # 定义目标方向角度（z轴方向）为欧拉角
    euler_angles = (pi/2, pi, 3*pi/2, 0)
    
    # 将角度变换成四元数
    for angle in euler_angles:
        q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
        q = Quaternion(*q_angle)    #   转换成ROS下四元数的表达方式
        quaternions.append(q)
    
    # 创建一个列表来保存waypoint poses
    self.waypoints = list()
            
    # Append each of the four waypoints to the list.  Each waypoint
    # is a pose consisting of a position and orientation in the map frame.
    #   起点
    self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
    #   第一个waypoint
    self.waypoints.append(Pose(Point(self.square_size, 0.0, 0.0), quaternions[0]))
    #   第二个waypoint
    self.waypoints.append(Pose(Point(self.square_size, self.square_size, 0.0), quaternions[1]))
    #   第三个waypoint
    self.waypoints.append(Pose(Point(0.0, self.square_size, 0.0), quaternions[2]))
    
    # 创建房间名并映射到waypoint的位置
    room_locations = (('explain_faith', self.waypoints[0]),
                      ('explanatory_text', self.waypoints[1]),
                      ('explain_the_rule', self.waypoints[2]),
                      ('explain_life', self.waypoints[3]))
    
    # 将映射存储为有序字典，以便我们可以按顺序访问房间
    self.room_locations = OrderedDict(room_locations)
    
    # 扩展底座（docking station）的位置
    self.docking_station_pose = (Pose(Point(0.5, 0.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))            
    
    # 初始化waypoint在RViz下可视化的标记
    init_waypoint_markers(self)
    
    # 设置每个waypoint在RViz下的可视化标记      
    for waypoint in self.waypoints:           
        p = Point()
        p = waypoint.position
        self.waypoint_markers.points.append(p)
        
    # 设置扩展底座（docking station）在RViz下的标记
    init_docking_station_marker(self)
        
    # 发布机器人控制指令 (e.g. to stop it)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    
    rospy.loginfo("Starting Tasks")
    
    # 发布waypoint markers
    self.marker_pub.publish(self.waypoint_markers)
    rospy.sleep(1)
    self.marker_pub.publish(self.waypoint_markers)
    
    # 发布docking station marker
    self.docking_station_marker_pub.publish(self.docking_station_marker)
    rospy.sleep(1)

def init_waypoint_markers(self):
    # Set up our waypoint markers
    marker_scale = 0.2
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
    
    # Define a marker publisher.
    self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
    
    # Initialize the marker points list.
    self.waypoint_markers = Marker()
    self.waypoint_markers.ns = marker_ns
    self.waypoint_markers.id = marker_id
    self.waypoint_markers.type = Marker.CUBE_LIST
    self.waypoint_markers.action = Marker.ADD
    self.waypoint_markers.lifetime = rospy.Duration(marker_lifetime)
    self.waypoint_markers.scale.x = marker_scale
    self.waypoint_markers.scale.y = marker_scale
    self.waypoint_markers.color.r = marker_color['r']
    self.waypoint_markers.color.g = marker_color['g']
    self.waypoint_markers.color.b = marker_color['b']
    self.waypoint_markers.color.a = marker_color['a']
    
    self.waypoint_markers.header.frame_id = 'odom'
    self.waypoint_markers.header.stamp = rospy.Time.now()
    self.waypoint_markers.points = list()

def init_docking_station_marker(self):
    # Define a marker for the charging station
    marker_scale = 0.3
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 0.7, 'g': 0.7, 'b': 0.0, 'a': 1.0}
    
    self.docking_station_marker_pub = rospy.Publisher('docking_station_marker', Marker, queue_size=5)
    
    self.docking_station_marker = Marker()
    self.docking_station_marker.ns = marker_ns
    self.docking_station_marker.id = marker_id
    self.docking_station_marker.type = Marker.CYLINDER
    self.docking_station_marker.action = Marker.ADD
    self.docking_station_marker.lifetime = rospy.Duration(marker_lifetime)
    self.docking_station_marker.scale.x = marker_scale
    self.docking_station_marker.scale.y = marker_scale
    self.docking_station_marker.scale.z = 0.02
    self.docking_station_marker.color.r = marker_color['r']
    self.docking_station_marker.color.g = marker_color['g']
    self.docking_station_marker.color.b = marker_color['b']
    self.docking_station_marker.color.a = marker_color['a']
    
    self.docking_station_marker.header.frame_id = 'odom'
    self.docking_station_marker.header.stamp = rospy.Time.now()
    self.docking_station_marker.pose = self.docking_station_pose
