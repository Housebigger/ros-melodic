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
        
        rospy.init_node('bigmama0306_rolling', anonymous=True)
        
        arm = moveit_commander.MoveGroupCommander('arm_bigmama')  
        
        arm.set_goal_joint_tolerance(0.001)

        raw_input(" Press Enter to Start...")
      
        body_name = 'robot::dummy'
        reference_frame = 'robot::dummy'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0.0, z = -0.1) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -80), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0.0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 20, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        raw_input("Dummy Force Applied, Press Enter to Continue...")

        arm.set_named_target('StartState')
        arm.go()
        rospy.sleep(0)
        arm.stop()
        
        raw_input("StartState Reached, Press Enter to Continue...")

        clear_body_wrench_client(body_name)
        
        #Starting rollover motion
        body_name = 'robot::Link_3'
        reference_frame = 'robot::Link_3'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = 0) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -50), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 50, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        raw_input("Link_3 Forced Applied, Press Enter to Continue...")   
        body_name = 'robot::dummy'
        reference_frame = 'robot::dummy'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = 0) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 1), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 2, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration) 

        body_name = 'robot::Link_3'
        reference_frame = 'robot::Link_3'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = -0.25) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 10, z = -50), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 20, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        joint_positions = [0, -2.530729, 0, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        joint_positions = [-0.2617995, -2.617995, -0.2617995, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        joint_positions = [-0.523599, -2.530729, -0.523599, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        joint_positions = [-0.785399, -2.356196, -0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        raw_input("Taking off successfully, Press Enter to Climb up...") 
        body_name = 'robot::Link_3'
        reference_frame = 'robot::Link_3'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = -0.25) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 10, z = -50), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 20, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        joint_positions = [-0.785399, -1.570797, -0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        raw_input("Ready for SnakeHand, Press Enter to Continue...")
        body_name = 'robot::Link_3'
        reference_frame = 'robot::Link_3'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = -0.25) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 10, z = -50), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 20, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        joint_positions = [2.356196, -1.570797, -0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)   

        joint_positions =  [1.570797, 0, 0, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)
        raw_input("Reached ZiQing Dun, Press Enter to Continue...")

        body_name = 'robot::Link_3'
        reference_frame = 'robot::Link_3'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = -0.25) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 10, z = -50), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 10, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        body_name = 'robot::dummy'
        reference_frame = 'robot::dummy'
        reference_point = geometry_msgs.msg.Point( x = 0, y = 0, z = 0) #(x = 0.1, y = 0.4, z = 0.02)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 1), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 3, nsecs = 0)
        duration = rospy.Duration(secs = 10, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        

        joint_positions = [0.523599, 2.094396, -0.523599, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        arm.stop()

        raw_input("EndState Reached, Press Enter to Continue>>>")
        body_name = 'robot::dummy'
        clear_body_wrench_client(body_name)
        body_name = 'robot::Link_3'
        clear_body_wrench_client(body_name)
        
        body_name = 'robot::dummy'
        reference_frame = 'robot::dummy'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = -80), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 100, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        raw_input("Dummy Force applied, Press Enter to Continue>>>")

        joint_positions = [-0.785399, 2.356196, -0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        joint_positions = [-0.785399, 1.570797, 2.356196, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

       

        joint_positions = [0, 0, 1.570797, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(2)

        joint_positions = [0.785399, -2.356196, 0.785399, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(0)

        arm.stop()

        raw_input("Ready to Go Home, Press Enter to Continue!!!")
             
        joint_positions = [-0.523599, -2.094396, 0.523599, 0]
        arm.set_joint_value_target(joint_positions)
                 
       
        arm.go()
        rospy.sleep(1)
        arm.stop()
      
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    
    try:
        MoveItBigmamaDemo()
    except rospy.ROSInterruptException:
        pass
