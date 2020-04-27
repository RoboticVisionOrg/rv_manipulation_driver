#!/usr/bin/env python

import rospy
import actionlib
import os

from math import pi

import moveit_commander
from moveit_commander.conversions import pose_to_list, list_to_pose

import geometry_msgs.msg
from std_msgs.msg import Empty
from moveit_msgs.msg import MoveGroupActionResult

class ManipulationMoveItDriver(object):
  def __init__(self, group_name=None):
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.groups = {}
    self.active_group = None
    self.set_group(group_name)

    self.result = None

    rospy.Subscriber('/move_group/result', MoveGroupActionResult, self._result_cb)

  def set_group(self, group_name):
    self.active_group = group_name
    if group_name is None:
        self.active_group = None
        return
    else:
        if group_name not in self.groups:
            if group_name not in self.robot.get_group_names():
                raise ValueError('Group name %s is not valid. Options are %s' % (group_name, self.robot.get_group_names()))
            self.groups[group_name] = moveit_commander.MoveGroupCommander(group_name)
        self.active_group = self.groups[group_name]

  def get_joint_values(self, group_name=None):
    if group_name:
        self.set_group(group_name)
    if not self.active_group:
        raise ValueError('No active Planning Group')
    return self.active_group.get_current_joint_values()

  def goto_joints(self, joint_values, velocity=None, group_name=None, wait=True):
    if group_name:
        self.set_group(group_name)
    if not self.active_group:
        raise ValueError('No active Planning Group')

    joint_goal = self.active_group.get_current_joint_values()
    if len(joint_goal) != len(joint_values):
        raise IndexError('Expected %d Joint Values, got %d' % (len(joint_goal), len(joint_values)))
    for i, v in enumerate(joint_values):
        joint_goal[i] = v

    self.active_group.set_max_velocity_scaling_factor(velocity if velocity else 0.5)
    
    self.result = None
    self.active_group.go(joint_goal, wait=False)

    if not wait:
      return True

    while not self.result:
      rospy.sleep(0.01)

    self.active_group.stop()
    self.active_group.clear_pose_targets()
    return self.result.result.error_code.val == 1


  def goto_pose(self, pose, velocity=None, group_name=None, wait=True):
    if group_name:
        self.set_group(group_name)
    if not self.active_group:
        raise ValueError('No active Planning Group')

    if type(pose) is list:
        pose = list_to_pose(pose)
    
    self.active_group.set_max_velocity_scaling_factor(velocity if velocity else 0.5)
    self.active_group.set_pose_target(pose)

    self.result = None
    self.active_group.go(wait=False)

    if not wait:
      return True

    while not self.result:
      rospy.sleep(0.01)

    self.active_group.stop()
    self.active_group.clear_pose_targets()
    return self.result.result.error_code.val == 1

  def goto_pose_cartesian(self, pose, velocity=None, group_name=None, wait=True):
    if group_name:
        self.set_group(group_name)
    if not self.active_group:
        raise ValueError('No active Planning Group')

    if type(pose) is list:
        pose = list_to_pose(pose)

    (plan, fraction) = self.active_group.compute_cartesian_path(
                                        [pose],   # waypoints to follow
                                        0.005,        # eef_step
                                        0.0)         # jump_threshold
    plan = self.active_group.retime_trajectory(self.robot.get_current_state(), plan, velocity if velocity else 0.5)

    if fraction != 1.0:
        raise ValueError('Unable to plan entire path!')
        return False

    self.result = None
    self.active_group.execute(plan, wait=False)
    
    if not wait:
      return True

    while not self.result:
      rospy.sleep(0.01)

    self.active_group.stop()
    self.active_group.clear_pose_targets()
    return self.result.result.error_code.val == 1


  def goto_named_pose(self, pose_name, velocity=None, group_name=None, wait=True):
    if group_name:
        self.set_group(group_name)
    if not self.active_group:
        raise ValueError('No active Planning Group')

    self.active_group.set_max_velocity_scaling_factor(velocity if velocity else 0.5)
    self.active_group.set_named_target(pose_name)
    
    self.result = None
    self.active_group.go(wait=False)

    if not wait:
      return True

    while not self.result:
      rospy.sleep(0.01)

    self.active_group.stop()
    self.active_group.clear_pose_targets()
    return self.result.result.error_code.val == 1


  def get_planner_ee_link(self, group_name=None):
    if group_name:
        self.set_group(group_name)
    if not self.active_group:
        raise ValueError('No active Planning Group')

    return self.active_group.get_end_effector_link()

  def get_pose_ik(self, pose, group_name=None):
    if group_name:
        self.set_group(group_name)
    if not self.active_group:
        raise ValueError('No active Planning Group')

    self.active_group.set_joint_value_target(pose)
    joints = self.active_group.get_joint_value_target()
    self.active_group.clear_pose_targets()

    return joints

  def set_gripper(self, width, speed=0.1, wait=True):
    raise NotImplementedError()

  def grasp(self, width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
    raise NotImplementedError()

  def stop(self):
    if self.active_group:
        self.active_group.stop()

  def recover(self):
    pass

  def _result_cb(self, msg):
    self.result = msg

