#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import time
import geometry_msgs.msg


import transforms3d as tfs
import numpy as np 
import math

from math import pi
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_tutorial',
    #                anonymous=True)
    self.x=0
    self.y=0
    self.z=0.69

    self.roll=0
    self.pitch=0
    self.yaw=0
    self.ms=1
    # self.ww=1
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "armgroup"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    rospy.init_node("listenerjoy",anonymous=True)
    rospy.Subscriber("/joy",Joy,self.callback)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
  def test(self):
    print("12121212")

  def callback(self,data):
    self.go_to_pose_goal(data)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
  def listener(self):
    rospy.spin()
  def go_to_joint_state(self):
    group = self.group
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[2] = joint_goal[2]+0.1
    print(str(joint_goal[0]))
    joint_goal[1] = -pi/4-0.8
    joint_goal[0] = 0.1
    joint_goal[3] = 3.14
    joint_goal[4] = 0
    joint_goal[5] = 1.53
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    #return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_statei_2(self):
    group = self.group
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -1.544
    joint_goal[1] = -1.732
    joint_goal[2] = 0.190
    joint_goal[3] = 3.14
    joint_goal[4] = 0
    joint_goal[5] = 1.53
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
  def go_to_joint_statei_3(self):
    group = self.group
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0.5
    joint_goal[1] = -pi/4
    joint_goal[2] = 1.4
    joint_goal[3] = -pi/2-0.4
    joint_goal[4] = -1.6
    joint_goal[5] = pi/3+0.6
    group.go(joint_goal, wait=True)

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4-0.5
    joint_goal[2] = 1.4
    joint_goal[3] = -pi/2-0.4
    joint_goal[4] = -1.6
    joint_goal[5] = pi/3+0.6
    group.go(joint_goal, wait=True)

    joint_goal[0] = -0.8
    joint_goal[1] = -pi/4
    joint_goal[2] = 1.4
    joint_goal[3] = -pi/2-0.4
    joint_goal[4] = -1.6
    joint_goal[5] = pi/3+0.6
    group.go(joint_goal, wait=True)


    joint_goal[0] = -1.6
    joint_goal[1] = -pi/4-0.5
    joint_goal[2] = 1.4
    joint_goal[3] = -pi/2-0.4
    joint_goal[4] = -1.6
    joint_goal[5] = pi/3+0.6
    group.go(joint_goal, wait=True)

    joint_goal[0] = -2.4
    joint_goal[1] = -pi/4
    joint_goal[2] = 1.4
    joint_goal[3] = -pi/2-0.4
    joint_goal[4] = -1.6
    joint_goal[5] = pi/3+0.6
    group.go(joint_goal, wait=True)

    joint_goal[0] = 0
    joint_goal[1] = -pi/4-0.5
    joint_goal[2] = 1.4
    joint_goal[3] = -pi/2-0.4
    joint_goal[4] = -1.6
    joint_goal[5] = pi/3+0.6
    group.go(joint_goal, wait=True)

    joint_goal[0] = 1.0
    joint_goal[1] = -pi/4
    joint_goal[2] = 1.4
    joint_goal[3] = -pi/2-0.4
    joint_goal[4] = -1.6
    joint_goal[5] = pi/3+0.6
    group.go(joint_goal, wait=True)

    joint_goal[0] = 1.5
    joint_goal[1] = -pi/4-0.8
    joint_goal[2] = 1.4
    joint_goal[3] = -pi/2-0.4
    joint_goal[4] = -1.6
    joint_goal[5] = pi/3+0.6
    group.go(joint_goal, wait=True)

    # parameters if you have already set the pose or joint target for the group

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL


  def go_to_pose_goal(self,data):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    #pose_goal.position.x = self.x+0.01
    #self.x+=0.01

    rpylist=group.get_current_rpy()

    if data.buttons[0]==1:
        self.x+=0.05*self.ms
    elif data.buttons[2]==1:
        self.x-=0.05*self.ms
    elif data.buttons[1]==1:
        self.y+=0.05*self.ms
    elif data.buttons[3]==1:
        self.y-=0.05*self.ms
    elif data.buttons[5]==1:
        self.z+=0.05*self.ms
    elif data.buttons[7]==1:
        self.z-=0.05*self.ms

    elif data.axes[0]==1:
        self.roll=rpylist[0]+0.2*self.ms
    elif data.axes[0]==-1:
        self.roll = rpylist[0]-0.2*self.ms
    elif data.axes[1]==1:
        self.pitch = rpylist[1]+0.2*self.ms
    elif data.axes[1]==-1:
        self.pitch = rpylist[1]-0.2*self.ms
    elif data.buttons[4]==1:
        self.yaw = rpylist[2]+0.2*self.ms
    elif data.buttons[6]==1:
        self.yaw = rpylist[2]-0.2*self.ms
    elif data.buttons[8]==1:
        self.ms+=1
    elif data.buttons[9]==1:
        self.ms-=1
    print("speed mode is :"+str(self.ms))
    pose_goal.position.y = self.y
    pose_goal.position.x = self.x
    
    pose_goal.position.z = self.z


    if self.yaw!=0 or self.pitch!=0 or self.roll!=0:
        quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

    #pose_goal.orientation.x = self.xx
    #pose_goal.orientation.y = self.yy
    #pose_goal.orientation.z = self.zz
    #pose_goal.orientation.w = self.ww
    print(str(self.x)+"+"+str(self.y)+"+"+str(self.z))
    print(group.get_current_rpy())
    group.set_pose_target(pose_goal)
    #plan = group.go(wait=True)
    plan = group.go()
    print('511')


    ## Now, we call the planner to compute the plan and execute it.
    # Calling `stop()` ensures that there is no residual movement
    #group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    #group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    #return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal2(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    #rpylist = [1.5473580717554953, -0.1037770361431429, -0.09370694102821339]
    # rpylist=[1.7953519056985934, -0.21404165682755383, -2.1171445214112166]
    # pose_goal = geometry_msgs.msg.Pose()

    # quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    #quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
    # pose_goal.orientation.x = 0.323
    # pose_goal.orientation.y = -0.71
    # pose_goal.orientation.z = -0.499
    # pose_goal.orientation.w = 0.3766

    # pose_goal.position.x = -0.182
    # pose_goal.position.y = -0.3352

    #joint4 (B) =-15du
    #rpylist=[-0.26616279247037966, 1.1542253280951234e-05, 0.0001531283688354627]
    pose_goal = geometry_msgs.msg.Pose()

    #quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    pose_goal.orientation.x = -1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0

    pose_goal.position.x = 0
    pose_goal.position.y = 0.62
    pose_goal.position.z = 0.3

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    #plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement

    #print "pose 1"
    time.sleep(1)

    #joint4B=-45du
    #rpylist=[-0.7807111133278347, -5.230105891264857e-05, -0.00011975742824927561]
    pose_goal = geometry_msgs.msg.Pose()

    #quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    #quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
    pose_goal.orientation.x = -1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0

    pose_goal.position.x = 0
    pose_goal.position.y = 0.62
    pose_goal.position.z = 0.2

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)

    #print "pose 2"
    time.sleep(1)


    #joint4B=20du
    #rpylist=[0.3459682326353647, -2.9834140813660265e-06, -5.146319937571067e-05]
    pose_goal = geometry_msgs.msg.Pose()

    #quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    #quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
    pose_goal.orientation.x = -1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0

    pose_goal.position.x = 0
    pose_goal.position.y = 0.62
    pose_goal.position.z = 0.1

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)

    #print "pose 3"
    time.sleep(1)


    group.stop()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal3(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    #rpylist = [1.5473580717554953, -0.1037770361431429, -0.09370694102821339]
    # rpylist=[1.7953519056985934, -0.21404165682755383, -2.1171445214112166]
    # pose_goal = geometry_msgs.msg.Pose()

    # quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    #quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
    # pose_goal.orientation.x = 0.323
    # pose_goal.orientation.y = -0.71
    # pose_goal.orientation.z = -0.499
    # pose_goal.orientation.w = 0.3766

    # pose_goal.position.x = -0.182
    # pose_goal.position.y = -0.3352

    #joint4 (B) =-15du
    #rpylist=[-0.26616279247037966, 1.1542253280951234e-05, 0.0001531283688354627]
    #pose_goal = geometry_msgs.msg.Pose()

    #quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    # pose_goal.orientation.x = -1
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.orientation.w = 0

    # pose_goal.position.x = 0
    # pose_goal.position.y = 0.45
    # pose_goal.position.z = 0.3

    # group.set_pose_target(pose_goal)
    # plan = group.go(wait=True)
   
    # time.sleep(1)

    # #pose1
    # pose_goal = geometry_msgs.msg.Pose()

    # #quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    # pose_goal.orientation.x = 0.9914
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.orientation.w = -0.1305

    # pose_goal.position.x = 0
    # pose_goal.position.y = 0.6
    # pose_goal.position.z = 0.08

    # group.set_pose_target(pose_goal)
    # plan = group.go(wait=True)
   
    # time.sleep(6)

    # #pose2
    # pose_goal = geometry_msgs.msg.Pose()

    # #quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    # pose_goal.orientation.x = 0
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.orientation.w = 1

    # pose_goal.position.x = 0
    # pose_goal.position.y = 0
    # pose_goal.position.z = 1.1920002

    # group.set_pose_target(pose_goal)
    # plan = group.go(wait=True)
   
    # time.sleep(6)

    #pose3
    pose_goal = geometry_msgs.msg.Pose()

    #quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    pose_goal.orientation.x = -0.7182415
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0.6957938356

    pose_goal.position.x = 0
    pose_goal.position.y = 0.76664010
    pose_goal.position.z = 0.1468453

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
   
    time.sleep(6)

    group.stop()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def go_to_pose_goal4(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    #rpylist = [1.5473580717554953, -0.1037770361431429, -0.09370694102821339]
    # rpylist=[1.7953519056985934, -0.21404165682755383, -2.1171445214112166]
    # pose_goal = geometry_msgs.msg.Pose()

    # quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    #quaternion = quaternion_from_euler(self.roll, self.pitch, self.yaw)
    # pose_goal.orientation.x = 0.323
    # pose_goal.orientation.y = -0.71
    # pose_goal.orientation.z = -0.499
    # pose_goal.orientation.w = 0.3766

    # pose_goal.position.x = -0.182
    # pose_goal.position.y = -0.3352

    #joint4 (B) =-15du
    #rpylist=[-0.26616279247037966, 1.1542253280951234e-05, 0.0001531283688354627]
    pose_goal = geometry_msgs.msg.Pose()

    #quaternion = quaternion_from_euler(rpylist[0], rpylist[1], rpylist[2])
    pose_goal.orientation.x = -1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0

    pose_goal.position.x = 0
    pose_goal.position.y = 0.436
    pose_goal.position.z = 0.12

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
   
    time.sleep(1)

    group.stop()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.01)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

pose_tag = None
def apriltag_pose_CB(msg):
  if len(msg.detections) > 0:
    print(msg)
    pose_tag = msg

def main():
  # ROS Stuff
  tutorial = MoveGroupPythonIntefaceTutorial()

  # moveit_commander.roscpp_initialize(sys.argv)

  # robot = moveit_commander.RobotCommander()

  # scene = moveit_commander.PlanningSceneInterface()

  # group_name = "armgroup"
  # group = moveit_commander.MoveGroupCommander(group_name)
  # rospy.init_node("listenerjoy",anonymous=True)

  group = tutorial.group

  end_effector_link = group.get_end_effector_link()
  curr_pose = group.get_current_pose(end_effector_link).pose
  print(curr_pose)

  from apriltag_ros.msg import AprilTagDetectionArray
  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_pose_CB, queue_size=1)
  

  # calculate minipulation matrix
  R_67=np.asarray([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
  P_67=np.asarray([0.,-0.1818,-0.16])
  T_67=tfs.affines.compose(P_67,R_67,[1,1,1])
  T_76=np.linalg.inv(T_67)

  T_Cend = pose_tag

  T_06 = curr_pose

  T6C = np.affines.compose(position,rotation,[1,1,1])

  T_0end=np.dot(np.dot(T_06,T_6C),T_Cend)

  T_mani = np.dot(T_0end,T76)

  tutorial.go_to_pose_goal3(T_mani)

  rospy.spin()

  return

  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."

    #raw_input("Press enter key to continue...")

    #tutorial.go_to_pose_goal2()
    #tutorial.listener()

    #raw_input("Press enter key to continue...")
    tutorial.go_to_pose_goal3()

    time.sleep(1)
    print "Motion perform finish."

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  #main()
  #while True:
      main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/kinetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL

