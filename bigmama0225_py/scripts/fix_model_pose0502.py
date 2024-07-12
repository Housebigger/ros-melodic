#!/usr/bin/env python
#coding=utf-8

from gazebo_msgs.srv import GetModelState, SetModelState
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import *

# Initialize ROS node
rospy.init_node('set_model_state_fixed')

# Wait for Gazebo services to start
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')

# Create GetModelState service proxy
get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# Create SetModelState service proxy
set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

# Get the current state of the model
model_name = 'robot'  # Replace with your model name
reference_frame= 'world'
current_state = get_model_state_proxy(model_name,reference_frame)

# Set the new model state to keep the model fixed in its current position
model_state = ModelState()
model_state.model_name = model_name
model_state.pose = current_state.pose  # Keep the current position
# Set the linear and angular velocities of the model to zero
#model_state.twist = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
model_state.twist.linear.x = 0.0
model_state.twist.linear.y = 0.0
model_state.twist.linear.z = 0.0
model_state.twist.angular.x = 0.0
model_state.twist.angular.y = 0.0
model_state.twist.angular.z = 0.0
model_state.reference_frame = 'world'

# Call the SetModelState service to set the model state
set_model_state_proxy(model_state)

# Output result
print("Model state set successfully, model is now fixed in its current position.")

# To ensure that the model stays fixed in its current position in future updates, we need to continuously call the SetModelState service
# This can be done using a loop or other logic based on your specific requirements
while not rospy.is_shutdown():
    set_model_state_proxy(model_state)
    rospy.sleep(0.01)  # Sleep for 0.01 second, adjust frequency as needed