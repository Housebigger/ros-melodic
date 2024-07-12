#!/usr/bin/env python
#coding=utf-8
 

 
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
 
class MoveItCartesianDemo:
    def __init__(self):
 
        # move_group API
        moveit_commander.roscpp_initialize(sys.argv)
 
        # moveit_cartesian_demo
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
        
        
        cartesian = rospy.get_param('~cartesian', True)
                        
        
        arm = MoveGroupCommander('wjarm')
        
        
        arm.allow_replanning(True)
        
        
        arm.set_pose_reference_frame('base_link')
                
        
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)
        
        
        end_effector_link = arm.get_end_effector_link()
                                        
        
        
        start_pose = arm.get_current_pose(end_effector_link).pose
                
      
        waypoints = []
                
       
        if cartesian:
            waypoints.append(start_pose)
            
        
        wpose = deepcopy(start_pose)
        wpose.position.x -= 0
        wpose.position.y -= 0
        wpose.position.z -= 0.2
 
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
         
        
        wpose.position.x += 0.05
        wpose.position.y += 0.15
        wpose.position.z -= 0.15
          
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
 
        
        arm.set_named_target('up')
        arm.go()
        rospy.sleep(1)
        
        
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
