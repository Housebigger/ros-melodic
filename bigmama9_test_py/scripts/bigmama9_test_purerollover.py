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

class MoveItBigmamaDemo:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)

        
        rospy.init_node('bigmama9_test', anonymous=True)
 
        
        arm = moveit_commander.MoveGroupCommander('arm_bigmama')
        
        
        #gripper = moveit_commander.MoveGroupCommander('gripper')
        
        
        arm.set_goal_joint_tolerance(0.001)
        #gripper.set_goal_joint_tolerance(0.001)
        
       
       
        
        #Starting rollover motion
        body_name = 'robot::Link_3'
        reference_frame = 'robot::Link_3'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -100), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 95, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        arm.set_named_target('chdili990')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('ziqingdun900')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('up')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('yewendun900')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('houchebu990')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('xiayao666')
        arm.go()
        rospy.sleep(2)
        arm.stop()

        raw_input("Press Enter to Continue!!!")
        
        clear_body_wrench_client(body_name)

        body_name = 'robot::dummy'
        reference_frame = 'robot::dummy'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -100), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 95, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        arm.set_named_target('beida099')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('houyang009')
        arm.go()
        rospy.sleep(0)
        
        arm.set_named_target('up')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('tiaotou009')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('baiheliangchi099')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('startstate666')
        arm.go()
        rospy.sleep(2)

        arm.stop()

        raw_input("Press Enter to Continue!!!")




        #gripper.set_joint_value_target([0.01])
        #gripper.go()
        #rospy.sleep(1)
         
        
        joint_positions = [0, 0, 0]
        arm.set_joint_value_target(joint_positions)
                 
       
        arm.go()
        rospy.sleep(10)
        
      
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    
    try:
        MoveItBigmamaDemo()
    except rospy.ROSInterruptException:
        pass
