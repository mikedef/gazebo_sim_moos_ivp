#!/usr/bin/env python3

'''
    NAME: Michael DeFilippo
    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
    FILE: desired_to_thrust.py
    DATE: 2022-11-14
    NOTE: Node parse data coming in from protobuf_client into
          topics for ROS consumption

   Copyright MIT and author/s of software.
   This is unreleased BETA code. No permission is granted or
   implied to use, copy, modify, and distribute this software
   except by the author(s), or those designated by the author.
'''

import rospy

from protobuf_client.msg import Gateway
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Node(object):
    def __init__(self):
        self.desired_thrust = 0.0
        self.desired_rudder = 0.0
        self.rudder_ok = False
        self.thrust_ok = False
        self.rudder_key = rospy.get_param("~rudder_key", "DESIRED_RUDDER")
        self.thrust_key = rospy.get_param("~thrust_key", "DESIRED_THRUST")
        self.left_msg = Float32()
        self.right_msg = Float32()
        self.left_thrust_key = rospy.get_param("~left_thrust_key", "COMMANDED_THRUST_L")
        self.right_thrust_key = rospy.get_param("~right_thrust_key", "COMMANDED_THRUST_R")
        self.left_thrust = Float32()
        self.right_thrust = Float32()
        self.left_ok = False
        self.right_ok = False
        # scaling MOOS thrust/rudder from -100:100 to -1:1
        self.linear_scaling = 1.0
        self.angular_scaling = 1.0
        # placing limits on how much thrust we send to each thruster. Probably need to edit once we are comfortable with moving the USV around
        self.upper_thrust_limit = 1.0
        self.lower_thrust_limit = -1.0

        # Publishers
        self.pub_left_thrust = rospy.Publisher("left_cmd", Float32, queue_size=10)
        self.pub_right_thrust = rospy.Publisher("right_cmd", Float32, queue_size=10)
        #self.pub_to_gateway = rospy.Publisher("/send_to_gateway", Gateway, queue_size=100)

        # Subscribers
        self.s1 = rospy.Subscriber("/gateway_msg", Gateway, self.parse_gateway_cb)

    def parse_gateway_cb(self, msg):
        #rospy.loginfo("parse gateway: %s"%(msg.gateway_key))
        #rospy.loginfo("keys: %s, %s"%(self.thrust_key, self.rudder_key))

        ''' If subscribing directly to MOOS params: DESIRED_THRUST, DESIRED_RUDDER  '''
        if msg.gateway_key == self.thrust_key:
            self.desired_thrust = msg.gateway_double
            self.thrust_ok = True
        elif msg.gateway_key == self.rudder_key:
            self.desired_rudder = msg.gateway_double
            self.rudder_ok = True

        if self.thrust_ok and self.rudder_ok:
            self.thrust_ok = False
            self.rudder_ok = False

            self.left_msg.data = self.desired_thrust/self.linear_scaling
            self.right_msg.data = self.desired_thrust/self.linear_scaling
            self.left_msg.data += (self.desired_rudder/self.angular_scaling)
            self.right_msg.data += -1*(self.desired_rudder/self.angular_scaling)
            # Limit outputs
            if self.left_msg.data > self.upper_thrust_limit:
                self.left_msg.data = self.upper_thrust_limit
            if self.left_msg.data < self.lower_thrust_limit:
                self.left_msg.data = self.lower_thrust_limit
            if self.right_msg.data > self.upper_thrust_limit:
                self.right_msg.data = self.upper_thrust_limit
            if self.right_msg.data < self.lower_thrust_limit:
                self.right_msg.data = self.lower_thrust_limit

            #self.pub_left_thrust.publish(self.left_msg)
            #self.pub_right_thrust.publish(self.right_msg)

        ''' If using iGazeboVessel MOOS app for control (current implementation)  '''
        if msg.gateway_key == self.left_thrust_key:
            self.thrust_left = msg.gateway_double
            self.left_ok = True
        elif msg.gateway_key == self.right_thrust_key:
            self.thrust_right = msg.gateway_double
            self.right_ok = True
        if self.left_ok and self.right_ok:
            self.left_ok = False
            self.right_ok = False
            self.pub_left_thrust.publish(self.thrust_left)
            self.pub_right_thrust.publish(self.thrust_right)

if __name__ == '__main__':
    rospy.init_node('desired_to_thrust', anonymous=True)

    # init class
    node = Node()

    # Print init info
    rospy.loginfo("Publishing to %s and %s"%(node.pub_left_thrust.name,
                                             node.pub_right_thrust.name))
    rospy.loginfo("Subscribing to %s"%(node.s1.name))
    rospy.loginfo("Thrust Key: %s"%(node.thrust_key))
    rospy.loginfo("Rudder Key: %s"%(node.rudder_key))
    rospy.loginfo("L Thrust Key: %s"%(node.left_thrust_key))
    rospy.loginfo("R Thrust Key: %s"%(node.right_thrust_key))

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
