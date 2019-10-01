#!/usr/bin/env python

import rospy
import actionlib

from math import pi

import moveit_commander
from moveit_commander.conversions import pose_to_list, list_to_pose

import geometry_msgs.msg
from std_msgs.msg import Empty

class ManipulationMoveItCommander(object):
    def __init__(self, group_name=None):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.groups = {}
        self.active_group = None
        self.set_group(group_name)

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

    def goto_joints(self, joint_values, group_name=None, wait=True):
        if group_name:
            self.set_group(group_name)
        if not self.active_group:
            raise ValueError('No active Planning Group')

        joint_goal = self.active_group.get_current_joint_values()
        if len(joint_goal) != len(joint_values):
            raise IndexError('Expected %d Joint Values, got %d' % (len(joint_goal), len(joint_values)))
        for i, v in enumerate(joint_values):
            joint_goal[i] = v

        success = self.active_group.go(joint_goal, wait)
        self.active_group.stop()
        return success

    def goto_pose(self, pose, velocity=0.5, group_name=None, wait=True):
        if group_name:
            self.set_group(group_name)
        if not self.active_group:
            raise ValueError('No active Planning Group')

        if type(pose) is list:
            pose = list_to_pose(pose)
        
        self.active_group.set_max_velocity_scaling_factor(velocity)
        self.active_group.set_pose_target(pose)
        success = self.active_group.go(wait=wait)
        self.active_group.stop()
        self.active_group.clear_pose_targets()
        return success

    def goto_pose_cartesian(self, pose, velocity=0.5, group_name=None, wait=True):
        if group_name:
            self.set_group(group_name)
        if not self.active_group:
            raise ValueError('No active Planning Group')

        if type(pose) is list:
            pose = list_to_pose(pose)

        self.active_group.set_max_velocity_scaling_factor(velocity)
        (plan, fraction) = self.active_group.compute_cartesian_path(
                                           [pose],   # waypoints to follow
                                           0.005,        # eef_step
                                           0.0)         # jump_threshold
        if fraction != 1.0:
            raise ValueError('Unable to plan entire path!')
            return False

        success = self.active_group.execute(plan, wait=wait)
        self.active_group.stop()
        self.active_group.clear_pose_targets()
        return success


    def goto_named_pose(self, pose_name, velocity=0.5, group_name=None, wait=True):
        if group_name:
            self.set_group(group_name)
        if not self.active_group:
            raise ValueError('No active Planning Group')

        self.active_group.set_max_velocity_scaling_factor(velocity)
        self.active_group.set_named_target(pose_name)
        success = self.active_group.go(wait=wait)
        self.active_group.stop()
        return success

    def stop(self):
        if self.active_group:
            self.active_group.stop()