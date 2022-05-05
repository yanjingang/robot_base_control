#!/usr/bin/env python
# coding=utf-8
"""
    角速度标定（旋转360度）
"""
import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign
from transform_utils import quat_to_angle, normalize_angle
import PyKDL
from math import pi


class CalibrateAngular():
    def __init__(self):
        # 为节点指定一个名称 Give the node a name
        rospy.init_node('calibrate_angular', anonymous=False)

        # ros终止事件 Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # 里程计检查频率 How fast will we check the odometry values?
        self.rate = 10
        r = rospy.Rate(self.rate)

        # 测试角度为360度 The test angle is 360 degrees
        self.test_angle = 2*pi  # 这里注意，在ROS中使用的弧度，不能直接写360
        self.speed = 0.1  # 每秒0.1弧度 radians per second
        self.tolerance = 1  # 1度转换为弧度 degrees converted to radians
        self.odom_angular_scale_correction = 1  # 角刻度校正
        self.start_test = True

        # 移动指令topic Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # 底座坐标topic The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # 轮速计topic   The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # 初始化tf  Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # 给tf一些初始化时间    Give tf some time to fill its buffer
        rospy.sleep(2)

        # 确保可以监听到topic Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")

        reverse = 1
        while not rospy.is_shutdown():
            if self.start_test:
                # 从tf获取当前旋转角度  Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                rospy.loginfo("self.odom_angle: "+str(self.odom_angle))

                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                rospy.loginfo("errir: "+str(error))

                # 测试交替方向 Alternate directions between tests
                reverse = -reverse
                while abs(error) > self.tolerance and self.start_test:
                    if rospy.is_shutdown():
                        return
                    rospy.loginfo("*************************** ")
                    # 旋转机器人以减少误差  Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    rospy.loginfo("move_cmd.angular.z: " + str(move_cmd.angular.z))
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()

                    # 从tf获取当前旋转角度  Get the current rotation angle from tf
                    self.odom_angle = self.get_odom_angle()
                    rospy.loginfo("current rotation angle: " + str(self.odom_angle))

                    # 计算自上次测量以来我们已经走了多远    Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
                    rospy.loginfo("delta_angle: "+str(delta_angle))

                    # 加上目前为止我们的总角度  Add to our total angle so far
                    turn_angle += abs(delta_angle)
                    rospy.loginfo("turn_angle: "+str(turn_angle))

                    # 计算新的差值  Compute the new error
                    error = self.test_angle - turn_angle
                    rospy.loginfo("error: "+str(error))

                    # 存储当前角度以便下次比较  Store the current angle for the next comparison
                    last_angle = self.odom_angle

                # 停止机器人    Stop the robot
                self.cmd_vel.publish(Twist())

                # 更新状态标志  Update the status flag
                self.start_test = False
                params = {'start_test': False}
                dyn_client.update_configuration(params)

            rospy.sleep(0.5)

        # 停止机器人    Stop the robot
        self.cmd_vel.publish(Twist())

    def get_odom_angle(self):
        # 获取odom和基本帧之间变换    Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        # 将旋转从四元数转换为欧拉角 Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))

    def shutdown(self):
        # 关闭节点时停止机器人  Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        CalibrateAngular()
    except:
        rospy.loginfo("Calibration terminated.")
