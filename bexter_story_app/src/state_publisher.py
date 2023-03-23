

#! /usr/bin/env python

import rospy
import interfaces.msg
import sys
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class StatePublisher(object):
    def __init__(self):
        # Create 3 publishers for each joint
        self.joint_0_pub = rospy.Publisher('joint_0', Float64, queue_size=10)
        self.joint_1_pub = rospy.Publisher('joint_1', Float64, queue_size=10)
        self.joint_2_pub = rospy.Publisher('joint_2', Float64, queue_size=10)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.callback)

    def callback(self, data):
        # Get joint states
        positions = data.position
        self.joint_0_pub.publish(positions[0])
        self.joint_1_pub.publish(positions[1])
        self.joint_2_pub.publish(positions[2])

if __name__ == "__main__":
    rospy.init_node("state_publisher_node")
    state_publisher = StatePublisher()
    rospy.spin()    
   