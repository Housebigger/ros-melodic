#!/usr/bin/env python
#coding=utf-8
 
import rospy
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, SetLinkState, SetLinkStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
 
def get_current_pose(link_name):
    
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        request = GetLinkStateRequest()
        request.link_name = link_name
        request.reference_frame = 'world'
        response = get_link_state_service(request)
        return response.pose  
    except rospy.ServiceException as e:
        print("Failed to call get link state service: %s"%e)
 
def set_link_state(link_name, pose):
    
    rospy.wait_for_service('/gazebo/set_link_state')
    try:
        set_link_state_service = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        request = SetLinkStateRequest()  
        
        request.link_name = link_name
        
        
        request.pose = pose
        
        
        request.twist.linear.x = 0
        request.twist.linear.y = 0
        request.twist.linear.z = 0
        request.twist.angular.x = 0
        request.twist.angular.y = 0
        request.twist.angular.z = 0
        
        
        response = set_link_state_service(request)
        return response.success  
    except rospy.ServiceException as e:
        print("Failed to call set link state service: %s"%e)
 
if __name__ == '__main__':
   
    rospy.init_node('keep_link_static_example')
    
   
    link_name = 'robot::Link_3'  
    
    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
       
        current_pose = get_current_pose(link_name)
        if current_pose:
          
            success = set_link_state(link_name, current_pose)
            
          
            if success:
                print("Link '%s' set to static relative to the world successfully." % link_name)
            else:
                print("Failed to set link '%s' to static relative to the world." % link_name)
        else:
            print("Could not get the current pose of link '%s'." % link_name)
        
        rate.sleep()  