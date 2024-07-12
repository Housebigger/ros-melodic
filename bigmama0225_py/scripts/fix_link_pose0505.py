#!/usr/bin/env python
#coding=utf-8 
import rospy
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, SetLinkState, SetLinkStateRequest
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Header
 
def get_current_pose(link_name):
    
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        response = get_link_state_service(link_name, 'world')  # 直接传递参数，不需要创建GetLinkStateRequest对象
        return response.link_state.pose  # 注意返回的是link_state.pose
    except rospy.ServiceException as e:
        print("Failed to call get link state service: %s"%e)
 
def set_link_state(link_name, pose):
    
    rospy.wait_for_service('/gazebo/set_link_state')
    try:
        set_link_state_service = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        request = SetLinkStateRequest()
        request.link_name = link_name
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = 'world'
        request.pose = pose
        request.twist.linear.x = 0.0
        request.twist.linear.y = 0.0
        request.twist.linear.z = 0.0
        request.twist.angular.x = 0.0
        request.twist.angular.y = 0.0
        request.twist.angular.z = 0.0
        return set_link_state_service(request).success
    except rospy.ServiceException as e:
        print("Failed to call set link state service: %s"%e)
 
if __name__ == '__main__':
    rospy.init_node('keep_link_static_example')
    link_name = 'robot::Link_4'
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_pose = get_current_pose(link_name)
        if current_pose:
            success = set_link_state(link_name, current_pose)
            print("link '%s' set to static relative to the world{'success' if success else 'fail'}." % link_name)
        else:
            print("Could not get the current pose of link '%s' ." % link_name)
        rate.sleep()
       