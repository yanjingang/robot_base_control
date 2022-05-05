#!/usr/bin/env python
# coding=utf-8
"""
    线速度标定（测试直行1米）
"""

import rospy
from geometry_msgs.msg import Twist, Point
from math import copysign, sqrt, pow
import tf


class CalibrateLinear():
    def __init__(self):
        # 1.节点初始化
        # Give the node a name
        rospy.init_node('calibrate_linear', anonymous=False)
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # 2.参数配置
        # How fast will we check the odometry values?
        # 检查odom的频率
        self.rate = 10
        r = rospy.Rate(self.rate)
        # Set the distance to travel
        # 设置移动距离
        self.test_distance = 1.0  # 测试直行1米 meters
        self.speed = 1  # 米/每秒 meters per second
        self.tolerance = 0.01  # 容忍误差 meters
        self.odom_linear_scale_correction = 1.0
        self.start_test = True

        # 3.topic pub/sub初始化
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')
        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))

        # 4.调出rqt_reconfigure来控制测试
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
        self.position = Point()
        # Get the starting position from the tf transform between the odom and base frames
        # 从 odom 和 base 帧之间的 tf变换中获取起始位置
        self.position = self.get_position()
        x_start = self.position.x
        y_start = self.position.y
        move_cmd = Twist()
        while not rospy.is_shutdown():
            # Stop the robot by default
            # 默认停止机器人
            move_cmd = Twist()
            if self.start_test:
                print('---------------')
                # Get the current position from the tf transform between the odom and base frames
                # 获取当前的位置信息
                self.position = self.get_position()
                print(self.position)
                # Compute the Euclidean distance from the target point
                # 计算当前位置与起始位置的距离
                distance = sqrt(pow((self.position.x - x_start), 2) + pow((self.position.y - y_start), 2))
                print(distance)
                # Correct the estimated distance by the correction factor
                # 用修正系数修正估计距离
                distance *= self.odom_linear_scale_correction
                # How close are we?
                # 计算与目标位置的距离
                error = distance - self.test_distance
                print(error)
                # Are we close enough?
                # 如果已经到达目标位置，则停止
                if not self.start_test or abs(error) < self.tolerance or error > 0:
                    self.start_test = False
                    params = False
                    rospy.loginfo(params)
                else:
                    # If not, move in the appropriate direction
                    # 如果还没有到达，则继续前进，如果已经超出了目标位置，则控制电机反转，退回
                    move_cmd.linear.x = copysign(self.speed, -1 * error)
            else:
                self.position = self.get_position()
                x_start = self.position.x   #设定起始位置的x坐标
                y_start = self.position.y   #设定起始位置的y坐标
            # 发布控制Twist消息
            #print(move_cmd)
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())

    def get_position(self):
        # Get the current transform between the odom and base frames
        # 获取odom和base帧之间的当前变换
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans)

    def shutdown(self):
        # Always stop the robot when shutting down the node
        # 关闭节点前先停止机器人运动
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        CalibrateLinear()
        rospy.spin()
    except:
        rospy.loginfo("Calibration terminated.")
