#!/usr/bin/env python

from gazebo_msgs.srv import SetLinkState
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

# Initialize ROS node
rospy.init_node('check_set_link_state_support')

# Wait for Gazebo service to start
rospy.wait_for_service('/gazebo/set_link_state')

# Try to get the service proxy
try:
    set_link_state_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
except rospy.ServiceException as e:
    print("Failed to get service proxy: %s" % e)

# Check if the service supports the 'twist' parameter
if set_link_state_proxy.resolved_args is not None:
    args = set_link_state_proxy.resolved_args
    if 'twist' in args:
        print("The service supports the 'twist' parameter.")
    else:
        print("The service does not support the 'twist' parameter.")
else:
    print("Unable to determine if the service supports the 'twist' parameter.")
