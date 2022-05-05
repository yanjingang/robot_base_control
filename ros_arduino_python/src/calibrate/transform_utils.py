#!/usr/bin/env python
# coding=utf-8

""" 
    从turtlebot_node.py脚本中找到的两个方便的转换实用程序”
        turtlebot_node ROS package at:http://www.ros.org/wiki/turtlebot_node
"""

import PyKDL
from math import pi

# 四元数转欧拉角
def quat_to_angle(quat):
    # 输入四元素，转换为旋转矩阵
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    # 旋转矩阵到RPY角
    return rot.GetRPY()[2]

# 格式化ROS角度值
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res