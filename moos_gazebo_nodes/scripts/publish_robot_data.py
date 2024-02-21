#!/usr/bin/env python3

'''
    NAME: Michael DeFilippo
    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
    FILE: publish_robot_data.py
    DATE: 2023-01-31
    NOTE: Node to publish robot data rpy, heading, speed from various sources

   Copyright MIT and author/s of software.
   This is unreleased BETA code. No permission is granted or
   implied to use, copy, modify, and distribute this software
   except by the author(s), or those designated by the author.
'''

import rospy
import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Vector3Stamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Node(object):
    def __init__(self):

        # Injest Launch Params
        self.namespace = rospy.get_param("~namespace", 'wamv')
        self.root_namespace = rospy.get_param("~root_namespace", 'none')

        # Publishers
        # set to namespace
        self.pub_rpy = rospy.Publisher('/' + self.root_namespace + '/' + self.namespace + "/sensors/imu/imu/rpy", Vector3, queue_size=100)
        self.pub_heading = rospy.Publisher('/' + self.root_namespace + '/' + self.namespace + "/sensors/imu/imu/heading", Float32, queue_size=100)
        self.pub_gps_speed = rospy.Publisher('/' + self.root_namespace + '/' + self.namespace + "/sensors/gps/gps/speed", Float32, queue_size=100)
        self.pub_gt_speed = rospy.Publisher('/' + self.root_namespace + '/' + self.namespace + "/robot_localization/speed", Float32, queue_size=100)
        self.pub_gt_heading = rospy.Publisher('/' + self.root_namespace + '/' + self.namespace + "/robot_localization/heading", Float32, queue_size=100)
        self.pub_denied_speed = rospy.Publisher('/' + self.root_namespace + '/' + self.namespace + "/denied_localization/speed", Float32, queue_size=100)
        self.pub_denied_heading = rospy.Publisher('/' + self.root_namespace + '/' + self.namespace + "/denied_localization/heading", Float32, queue_size=100)

        # Subscribers
        self.s1 = rospy.Subscriber('/' + self.root_namespace + '/' + self.namespace + '/sensors/imu/imu/data', Imu, self.imu_cb)
        self.s2 = rospy.Subscriber('/' + self.root_namespace + '/' + self.namespace + '/sensors/gps/gps/fix_velocity', Vector3Stamped, self.speed_from_gps)
        self.s3 = rospy.Subscriber('/' + self.root_namespace + '/' + self.namespace + '/robot_localization/odometry/filtered', Odometry, self.gt_odom)
        self.s4 = rospy.Subscriber('/' + self.root_namespace + '/' + self.namespace + '/denied_localization/odometry/filtered', Odometry, self.denied_odom)


    def imu_cb(self, msg):
        q = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = 1.57 - yaw
        rpy = Vector3()
        rpy.x = roll
        rpy.y = pitch
        rpy.z = yaw
        heading = Float32()
        heading.data = math.fmod(math.degrees(yaw) + 360, 360)  # 0-360 degrees
        self.pub_rpy.publish(rpy)
        self.pub_heading.publish(heading)

    def speed_from_gps(self, msg):
        speed = Float32()
        calc = math.sqrt(math.pow(msg.vector.x,2) + math.pow(msg.vector.y,2))
        if calc < 0.05:
            calc = 0.0
        speed.data = calc
        self.pub_gps_speed.publish(speed)

    def gt_odom(self, msg):
        speed = Float32()
        heading = Float32()
        speed.data = self.calc_speed(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        heading.data = self.calc_heading(msg.pose.pose.orientation)
        self.pub_gt_speed.publish(speed)
        self.pub_gt_heading.publish(heading)

    def denied_odom(self, msg):
        speed = Float32()
        heading = Float32()
        speed.data = self.calc_speed(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        heading.data = self.calc_heading(msg.pose.pose.orientation)
        self.pub_denied_speed.publish(speed)
        self.pub_denied_heading.publish(heading)

    def calc_speed(self, x, y):
        calc = math.sqrt(math.pow(x,2) + math.pow(y,2))
        if calc < 0.05:
            calc = 0
        return calc

    def calc_heading(self, q):
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = 1.57 - yaw
        return math.fmod(math.degrees(yaw) + 360, 360)  # 0-360 degrees

if __name__ == '__main__':
    rospy.init_node('publish_robot_data', anonymous=True)

    # init class
    node = Node()

    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
