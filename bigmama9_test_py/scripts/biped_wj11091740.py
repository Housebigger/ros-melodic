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
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def clear_body_wrench_client(body_name):
        rospy.wait_for_service('gazebo/clear_body_wrenches')
        try:
            clear_body_wrench = rospy.ServiceProxy('gazebo/clear_body_wrenches', BodyRequest)
            clear_body_wrench(body_name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

class MoveItwjDemo:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)

        
        rospy.init_node('wj_test', anonymous=True)
 
        
        arm = moveit_commander.MoveGroupCommander('wjarm')
        
        
        #gripper = moveit_commander.MoveGroupCommander('gripper')
        
        
        arm.set_goal_joint_tolerance(0.001)
        #gripper.set_goal_joint_tolerance(0.001)
        
        #Starting inchworm motion
        body_name = 'robot::A_Link'
        reference_frame = 'robot::A_Link'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = -1)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -100), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 5, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        joint_positions = [0, -0.508066, 1.63869, 0.221831, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)
        
        arm.stop()
        raw_input("Press Enter to Continue!!!")

        clear_body_wrench_client(body_name)

        body_name = 'robot::D_Link'
        reference_frame = 'robot::D_Link'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = -1)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -100), torque = geometry_msgs.msg.Vector3( x = -20, y = -0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 10, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        joint_positions2 = [0, 0, 0, 0, 0]
        arm.set_joint_value_target(joint_positions2)
        arm.go()
        rospy.sleep(1)

        arm.stop()
        raw_input("Press Enter to Continue!!!")

        
        
        joint_positions3 = [0, -0.523599, 2.094396, 0.523599, 0]
        arm.set_joint_value_target(joint_positions3)
        arm.go()
        rospy.sleep(1)

        arm.stop()
        raw_input("Press Enter to Continue!!!")
        
        clear_body_wrench_client(body_name)

        body_name = 'robot::A_Link'
        reference_frame = 'robot::A_Link'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = -1)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -100), torque = geometry_msgs.msg.Vector3( x = -20, y = -0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 10, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        joint_positions4 = [0,  -0.523599, 0, -0.523599, 0]
        arm.set_joint_value_target(joint_positions4)
        arm.go()
        rospy.sleep(1)
        
        
        arm.stop()
        raw_input("Press Enter to Continue!!!")


        arm.set_named_target('up')
        arm.go()
        rospy.sleep(1)

        

        #gripper.set_joint_value_target([0.01])
        #gripper.go()
        #rospy.sleep(1)
         
        
        joint_positions5 = [0, 0, 0, 0, 0]
        arm.set_joint_value_target(joint_positions5)
        arm.go()
        rospy.sleep(1)
        clear_body_wrench_client(body_name)
      
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    
    try:
        MoveItwjDemo()
    except rospy.ROSInterruptException:
        pass
