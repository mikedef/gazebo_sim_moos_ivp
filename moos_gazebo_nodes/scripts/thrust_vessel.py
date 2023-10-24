#!/usr/bin/env python3                                                                                  

'''                                                                                                     
    NAME: Michael DeFilippo                                                                             
    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA                                             
    FILE: thrust_vessel.py                                                                        
    DATE: 2023-10-24                                                                                    
    NOTE: Node to parse data coming in from protobuf_client into                                 
          topics for ROS consumption          
                                                                                                        
 This is unreleased BETA code. no permission is granted or                                              
 implied to use, copy, modify, and distribute this software                                             
 except by the author(s), or those designated by the author.                                            
'''                                                                                                     
                                                                                                        

import rospy

from protobuf_client.msg import Gateway
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Node(object):
    def __init__(self):
        self.left_thrust_key = rospy.get_param("~left_thrust_key", "COMMANDED_THRUST_L")
        self.right_thrust_key = rospy.get_param("~right_thrust_key", "COMMANDED_THRUST_R")
        self.left_thrust = Float32()
        self.right_thrust = Float32()
        self.left_ok = False
        self.right_ok = False
        
        # Publishers
        self.pub_left_thrust = rospy.Publisher("left_cmd", Float32, queue_size=10)
        self.pub_right_thrust = rospy.Publisher("right_cmd", Float32, queue_size=10)
        
        # Subscribers
        self.s1 = rospy.Subscriber("/gateway_msg", Gateway, self.parse_gateway_cb)

    def parse_gateway_cb(self, msg):
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
    rospy.init_node('thrust_vessel', anonymous=True)

    # init class
    node = Node()

    # Print init info
    rospy.loginfo("Publishing to %s and %s"%(node.pub_left_thrust.name,
                                             node.pub_right_thrust.name))
    rospy.loginfo("Subscribing to %s"%(node.s1.name))
    rospy.loginfo("L Thrust Key: %s"%(node.left_thrust_key))
    rospy.loginfo("R Thrust Key: %s"%(node.right_thrust_key))

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
