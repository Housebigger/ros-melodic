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
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from control_msgs.msg import GripperCommand
from gazebo_msgs.srv import *
from copy import deepcopy

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
        
        
        arm.set_goal_joint_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)
        #gripper.set_goal_joint_tolerance(0.001)
        
        #Starting inchworm motion
        body_name = 'robot::A_Link'
        reference_frame = 'robot::A_Link'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 40, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        joint_positions = [1.570797, 0, -1.570797, -1.570797, -1.570797]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)
        
        arm.stop()
        raw_input("Press Enter to Continue!!!")
        
        #Cartesian lift
        cartesian = rospy.get_param('~cartesian', True)
        arm.allow_replanning(True)
        arm.set_pose_reference_frame('base_link')
        end_effector_link = arm.get_end_effector_link()
        
        start_pose = arm.get_current_pose(end_effector_link).pose
        waypoints = []

        if cartesian:
            waypoints.append(start_pose)
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.3
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)

        if cartesian:
            waypoints.append(start_pose)
        else:
            arm.set_pose_target(start_pose)
            arm.go()
            rospy.sleep(1)

        if cartesian:
            fraction = 0.0   
            maxtries = 100   
            attempts = 0     
            
            
            arm.set_start_state_to_current_state()
     
            
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = arm.compute_cartesian_path (
                                        waypoints,  
                                        0.01,        
                                        0.0,         
                                        True)       
                
                
                attempts += 1
                
                
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")


        arm.stop()
        raw_input("Press Enter to Continue!!!")

        joint_positions = [1.570797, 0.523599, -2.094396, -0.523599, -1.570797]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)
        
        arm.stop()
        raw_input("Press Enter to Continue!!!")

        clear_body_wrench_client(body_name)

        body_name = 'robot::D_Link'
        reference_frame = 'robot::D_Link'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 15, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)
        
        joint_positions = [0, 0, 0, 0, 1.570797]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)

        arm.stop()
        raw_input("Press Enter to Continue!!!")
        
        
        joint_positions = [1.570797, -0.523599, 2.094396, 0.523599, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)

        arm.stop()
        raw_input("Press Enter to Continue!!!")
        
        clear_body_wrench_client(body_name)

        body_name = 'robot::A_Link'
        reference_frame = 'robot::A_Link'
        reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
        wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0), torque = geometry_msgs.msg.Vector3( x = 0, y = 0, z = 0))
        start_time = rospy.Time(secs = 0, nsecs = 0)
        duration = rospy.Duration(secs = 13, nsecs = 0)
        apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration)

        joint_positions = [1.570797, 0.523599, -2.094396, -0.523599, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)
        
        
        arm.stop()
        raw_input("Press Enter to Continue!!!")


        arm.set_named_target('up')
        arm.go()
        rospy.sleep(2)

        

        #gripper.set_joint_value_target([0.01])
        #gripper.go()
        #rospy.sleep(1)
         
        
        joint_positions = [0, 0, 0, 0, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(1)
        
      
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    
    try:
        MoveItwjDemo()
    except rospy.ROSInterruptException:
        pass
