#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import GripperCommand
from gazebo_msgs.srv import *

def apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        apply_body_wrench(body_name, reference_frame, reference_point, wrench, start_time, duration)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def clear_body_wrench_client(body_name):
        rospy.wait_for_service('gazebo/clear_body_wrenches')
        try:
            clear_body_wrench = rospy.ServiceProxy('gazebo/clear_body_wrenches', BodyRequest)
            clear_body_wrench(body_name)
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)

class MoveItBigmamaDemo:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('bigma7ax_purewrench', anonymous=True)
        
        arm1 = moveit_commander.MoveGroupCommander('arm1_bigmama')  
        arm2 = moveit_commander.MoveGroupCommander('arm2_bigmama') 
        
        arm1.set_goal_joint_tolerance(0.001)
        arm2.set_goal_joint_tolerance(0.001)

        raw_input(" Press Enter to Start...")
        body_name = 'robot::dummy'
        reference_frame = 'robot::dummy'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0.0, z = 0) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 10), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0.0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 50, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        body_name = 'robot::Link_14'
        reference_frame = 'robot::Link_14'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0.0, z = 0) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 10), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0.0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 50, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        body_name = 'robot::Link_23'
        reference_frame = 'robot::Link_23'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0.0, z = 0) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -100), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0.0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 50, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        raw_input("Arm2 Link_23 Force Applied, Press Enter to Continue...")

        arm1.set_named_target('lift11')
        arm1.go()
        rospy.sleep(0)
        arm1.stop()

        arm2.set_named_target('lift21')
        arm2.go()
        rospy.sleep(0)
        arm2.stop()       

        raw_input("Arm1 lifted, Press Enter to Continue...")

        

        #joint_positions = [-0.785399, -1.570797, -0.785399, 0]
        #arm.set_joint_value_target(joint_positions)
        #arm.go()
        #rospy.sleep(0)
        #arm.stop()
      
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    
    try:
        MoveItBigmamaDemo()
    except rospy.ROSInterruptException:
        pass
