#!/usr/bin/env python
#coding=utf-8

from gazebo_msgs.srv import GetLinkState, SetLinkState
import rospy
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import *

# Initialize ROS node
rospy.init_node('set_link_state_fixed')

# Wait for Gazebo services to start
rospy.wait_for_service('/gazebo/get_link_state')
rospy.wait_for_service('/gazebo/set_link_state')

# Create GetLinkState service proxy
get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

# Create SetLinkState service proxy
set_link_state_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

# Get the current state of the link
link_name = 'Link_3'  # Replace with your link name
reference_frame= 'world'
current_state = get_link_state_proxy(link_name,reference_frame)

# Set the new link state to keep the link fixed in its current position
link_state = LinkState()
link_state.link_name = link_name
link_state.pose = current_state.link_state.pose  # Keep the current position
# Set the linear and angular velocities of the link to zero
#link_state.twist = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
link_state.twist.linear.x = 0.0
link_state.twist.linear.y = 0.0
link_state.twist.linear.z = 0.0
link_state.twist.angular.x = 0.0
link_state.twist.angular.y = 0.0
link_state.twist.angular.z = 0.0
link_state.reference_frame = 'world'

# Call the SetlinkState service to set the link state
set_link_state_proxy(link_state)

# Output result
print("link state set successfully, link is now fixed in its current position.")

# To ensure that the link stays fixed in its current position in future updates, we need to continuously call the SetlinkState service
# This can be done using a loop or other logic based on your specific requirements
while not rospy.is_shutdown():
    set_link_state_proxy(link_state)
    rospy.sleep(0.002)  # Sleep for 0.00002 second, adjust frequency as needed
