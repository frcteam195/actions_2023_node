#!/usr/bin/env python3

import tf2_ros
import rospy

def ros_main(node_name):
    rospy.init_node(node_name)

    #Your code here
    print("Test Print")

    rospy.spin()