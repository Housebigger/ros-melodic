#!/usr/bin/env python
#coding=utf-8
import rospy
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, SetLinkState, SetLinkStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

def get_current_pose(link_name):
    """
    Get the current pose of the specified link.
    
    :param link_name: The name of the link to get the pose for
    :return: The current pose of the link as a Pose object
    """
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        get_link_state_request = GetLinkStateRequest()
        get_link_state_request.link_name = link_name
        get_link_state_request.reference_frame = 'world'
        response = get_link_state_service(get_link_state_request)
        return response.pose
    except rospy.ServiceException as e:
        print("Failed to call get link state service: %s"%e)

def set_link_state(link_name, pose):
    """
    Set the state of the specified link to keep it static relative to the world.
    
    :param link_name: The name of the link to set the state for
    :param pose: The desired pose, here we set all position and orientation components to 0
    """
    rospy.wait_for_service('/gazebo/set_link_state')
    try:
        set_link_state_service = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        set_link_state_request = SetLinkStateRequest()
        
        # Set request header information
        set_link_state_request.header = Header()
        set_link_state_request.header.stamp = rospy.Time.now()
        set_link_state_request.header.frame_id = 'world'
        
        # Set the link name
        set_link_state_request.link_name = link_name
        
        # Set the target pose
        set_link_state_request.pose = pose
        
        # Set the angular and linear velocities to 0
        set_link_state_request.twist.linear.x = 0
        set_link_state_request.twist.linear.y = 0
        set_link_state_request.twist.linear.z = 0
        set_link_state_request.twist.angular.x = 0
        set_link_state_request.twist.angular.y = 0
        set_link_state_request.twist.angular.z = 0
        
        # Send the request and wait for the response
        response = set_link_state_service(set_link_state_request)
        return response.success
    except rospy.ServiceException as e:
        print("Failed to call set link state service: %s"%e)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('keep_link_static_example')
    
    # Set the link name to keep it static
    link_name = 'robot::Link_4'  # Replace with the name of the link you want to keep static
    
    rate = rospy.Rate(10)  # Set the loop frequency to 10Hz
    while not rospy.is_shutdown():
        # Get the current pose of the link
        current_pose = get_current_pose(link_name)
        if current_pose:
            # Create a Pose message with all position and orientation components set to the current values
            pose = Pose()
            pose.position = current_pose.pose.position
            pose.orientation = current_pose.pose.orientation
            
            # Call the function to set the link state
            success = set_link_state(link_name, pose)
            
            # Print the result
            if success:
                print("Link '%s' set to static relative to the world successfully." % link_name)
            else:
                print("Failed to set link '%s' to static relative to the world." % link_name)
        else:
            print("Could not get the current pose of link '%s'." % link_name)
        
        rate.sleep()  # Sleep based on the set loop frequency