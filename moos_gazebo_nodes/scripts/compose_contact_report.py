#!/usr/bin/env python3

"""
    NAME: Michael DeFilippo
    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
    FILE: compose_contact_report.py
    DATE: 2023-01-03
    NOTE: Node to parse contact vessel topics into a node report for MOOS
          consumption
    UPDATES: Updates with Aurora (Karan) 2024

 This is unreleased BETA code. no permission is granted or
 implied to use, copy, modify, and distribute this software
 except by the author(s), or those designated by the author.
"""


import rospy
import math
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3Stamped
from tf.transformations import euler_from_quaternion
from protobuf_client.msg import Gateway


class Node(object):
    def __init__(self):
        self.robot = Point()  # x, y, heading output from robot_localization pkg

        # Contact
        # Injest Launch Params
        self.contact_namespace = rospy.get_param("~namespace", "cora")
        self.contact_root_namespace = rospy.get_param("~root_namespace", "asset")
        self.contact_length = rospy.get_param("~contact_length", 1.0)
        self.contact_lat = 0.0
        self.contact_lon = 0.0
        self.contact_heading = 0.0
        self.contact_speed = 0.0
        self.contact_depth = 0.0
        self.contact_roll = 0.0
        self.contact_pitch = 0.0
        self.contact_yaw = 0.0
        self.contact_cog = 0.0  # course over ground
        self.gateway_key = "ASSET_NODE_REPORT_GAZEBO"  # rospy.get_param("~gateway_key", 'GROUNDTRUTH_CONTACT_REPORT')
        self.contact_gt = rospy.get_param("~contact_gt", False)
        self.contact_point_good = False  # bool for when an updated contact point comes in from sensor processing
        self.pub_topic = rospy.get_param("~pub_topic", "/send_to_gateway")

        # Publishers
        self.pub_contact_report = rospy.Publisher(self.pub_topic, Gateway, queue_size=1)

        # Subscribers
        if self.contact_gt:
            self.s1 = rospy.Subscriber(
                "/" + self.contact_root_namespace + "/" + self.contact_namespace + "/sensors/gps/gps/fix", NavSatFix, self.contact_gps_cb
            )
            self.s2 = rospy.Subscriber(
                "/" + self.contact_root_namespace + "/" + self.contact_namespace + "/sensors/gps/gps/fix_velocity", Vector3Stamped, self.speed_cb
            )
            self.s3 = rospy.Subscriber("/" + self.contact_root_namespace + "/" + self.contact_namespace + "/sensors/imu/imu/data", Imu, self.imu_cb)

    def imu_cb(self, msg):
        q = msg.orientation
        (self.contact_roll, self.contact_pitch, self.contact_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.contact_yaw = 1.57 - self.contact_yaw
        # Fix for MOOS consumption, needs to be -pi to pi
        self.contact_heading = math.fmod(math.degrees(self.contact_yaw) + 360, 360)  # 0-360 degrees

    def speed_cb(self, msg):
        """Calculate speed of vessel based on previous msg data"""
        self.contact_speed = math.sqrt(math.pow(msg.vector.x, 2) + math.pow(msg.vector.y, 2))
        if self.contact_speed <= 0.1:
            self.contact_speed = 0.0

        self.contact_cog = math.degrees(math.atan2(msg.vector.x, msg.vector.y)) - 90.0
        self.contact_cog = math.fmod(self.contact_cog + 360, 360)

    def contact_gps_cb(self, msg):
        self.contact_lat = msg.latitude
        self.contact_lon = msg.longitude
        self.contact_depth = 0.0  # msg.altitude
        self.contact_groundtruth()

    def contact_groundtruth(self):
        """Publish GPS as groundtruth"""
        # publish as a Gateway msg to be sent to MOOS
        contact_report = Gateway()
        time = rospy.Time.now()
        time_sec = time.to_sec()
        contact_report.header.stamp = time
        contact_report.gateway_time = time
        contact_report.gateway_key = self.gateway_key
        contact_report.gateway_string = (
            "NAME="
            + self.contact_root_namespace
            + ",TYPE=ship,TIME="
            + str(time_sec)
            + ",LAT="
            + str(self.contact_lat)
            + ",LON="
            + str(self.contact_lon)
            + ",HDG="
            + str(self.contact_heading)
            + ",COG="
            + str(self.contact_cog)
            + ",YAW="
            + str(self.contact_yaw)
            + ",PITCH="
            + str(self.contact_pitch)
            + ",ROLL="
            + str(self.contact_roll)
            + ",SPD="
            + str(self.contact_speed)
            + ",DEP="
            + str(self.contact_depth)
            + ",LENGTH="
            + str(self.contact_length)
        )

        # rospy.loginfo("Publishing: %s"%(contact_report.gateway_string))
        self.pub_contact_report.publish(contact_report)

    def contact_cb(self, msg):
        """publish sensor information as a contact report"""
        None
        # contact_report.gateway_key = 'CONTACT_REPORT'

    def iterate(self):
        # Bump up rate if we notice data is slow to get to pMarineViewer
        # **** Remove sleep for clustering algorithm. We want this to run every time there is an update from the sensor
        rate = rospy.Rate(4)  # 4 hz
        while not rospy.is_shutdown():
            # self.publish_contact_report()
            # Sleep for defined rate
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("compose_contact_report", anonymous=True)

    # init class
    node = Node()

    # Print init info
    rospy.loginfo("Compose_Contact_report: Publishing to %s" % (node.pub_contact_report.name))
    rospy.loginfo("Compose_Contact_report: Subscribing to %s, %s, %s and %s" % (node.s1.name, node.s2.name, node.s3.name, "None"))

    try:
        # node.iterate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
