#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import GripperCommand

class MoveItBigmamaDemo:
    def __init__(self):
        
        moveit_commander.roscpp_initialize(sys.argv)

        
        rospy.init_node('bigmama9_test', anonymous=True)
 
        
        arm = moveit_commander.MoveGroupCommander('arm_bigmama')
        
        
        #gripper = moveit_commander.MoveGroupCommander('gripper')
        
        
        arm.set_goal_joint_tolerance(0.001)
        #gripper.set_goal_joint_tolerance(0.001)
        
        
        arm.set_named_target('startstate666')
        arm.go()
        rospy.sleep(0)
         
        arm.set_named_target('baiheliangchi099')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('taishouxx9')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('endstate121212')
        arm.go()
        rospy.sleep(2)
        arm.stop()

        raw_input("Press Enter to Continue!!!")

        arm.set_named_target('xiagouquan9xx')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('chdili990')
        arm.go()
        rospy.sleep(0)

        arm.set_named_target('startstate666')
        arm.go()
        rospy.sleep(2)
        arm.stop()

        raw_input("Press Enter to Continue!!!")

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
