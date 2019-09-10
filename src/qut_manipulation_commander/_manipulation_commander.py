#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import actionlib
import importlib

from ._control_switcher import ControlSwitcher
from ._action_proxy import ActionProxy
from ._manipulation_moveit_commander import ManipulationMoveItCommander

from std_srvs.srv import Empty

from qut_manipulation_msgs.msg import MoveToPoseAction, MoveToPoseResult
from qut_manipulation_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseResult
from qut_manipulation_msgs.msg import MoveGripperAction

class ManipulationCommander(object):
  def __init__(self, moveit_commander=None):
    self.switcher = ControlSwitcher()
    
    # params
    self.controllers = rospy.get_param('~controllers', None)

    if not moveit_commander:
      self.move_group = rospy.get_param('~move_group', None)
    
    if not self.controllers:
      rospy.logerr('Unable to load controllers from rosparam server path: controllers')
      sys.exit(1)

    if not self.move_group:
      rospy.logerr('Unable to load move_group name from rosparam server path: move_group')
      sys.exit(1)

    self.action_proxies = []
    self.publishers = []

    for controller in self.controllers:
      if controller['type'] == 'publisher':
        self.publishers.append(self.create_publisher(
          controller['controller'],
          controller['name'],
          controller['maps'] if 'maps' in controller else None,
          controller['topic_type']
        ))
      
      if controller['type'] == 'action_server':
        self.action_proxies.append(self.create_action_server(
          controller['controller'],
          controller['name'],
          controller['maps'] if 'maps' in controller else None,
          controller['topic_type']
        ))
    
    self.moveit_commander = moveit_commander if moveit_commander else ManipulationMoveItCommander(group_name=self.move_group)

    self.pose_server = actionlib.SimpleActionServer(
      'cartesian/pose', 
      MoveToPoseAction,
      execute_cb=self.pose_cb,
      auto_start=False
    )

    self.location_server = actionlib.SimpleActionServer(
      'cartesian/named_pose',
      MoveToNamedPoseAction,
      execute_cb=self.location_cb,
      auto_start=False
    )

    self.gripper_server = actionlib.SimpleActionServer(
      '/gripper',
      MoveGripperAction,
      execute_cb=self.gripper_cb,
      auto_start=False
    )

    self.pose_server.start()
    self.location_server.start()
    self.gripper_server.start()

    rospy.Service('/home', Empty, self.home_cb)
    
  def create_publisher(self, controller_name, topic_in, topic_out, topic_type_name):
    topic_type = self.__get_topic_type(topic_type_name)

    if topic_out:
      _publisher = rospy.Publisher(topic_out, topic_type, queue_size=1)

    def _callback(msg):
      if self.switcher.get_current_name() != controller_name:
        self.switcher.switch_controller(controller_name)

      if topic_out:  
        _publisher.publish(msg)
    
    return rospy.Subscriber(topic_in, topic_type, _callback, queue_size=1)
    
  def pose_cb(self, goal):
    if self.switcher.get_current_name() != 'position_joint_trajectory_controller':
      self.switcher.switch_controller('position_joint_trajectory_controller')

    self.moveit_commander.stop()

    pose = [
      goal.pose.pose.position.x,
      goal.pose.pose.position.y, 
      goal.pose.pose.position.z,
      goal.pose.pose.orientation.x,
      goal.pose.pose.orientation.w,
      goal.pose.pose.orientation.z,
      goal.pose.pose.orientation.w
    ]
    
    self.moveit_commander.goto_pose(pose)
    self.pose_server.set_succeeded(MoveToPoseResult(result=0))

  def gripper_cb(self, goal):
    self.gripper_server.set_succeeded(MoveGripperResult(result=0))

  def location_cb(self, goal):
    self.__move_to_named(goal.named_pose)

  def home_cb(self, req):
    self.__move_to_named('ready')
    return []

  def __move_to_named(self, named):   
    if self.switcher.get_current_name() != 'position_joint_trajectory_controller':
      self.switcher.switch_controller('position_joint_trajectory_controller')
    
    self.moveit_commander.stop()
    return self.moveit_commander.goto_named_pose(named)
    
  def __get_topic_type(self, topic_type_name):
    module, typename = topic_type_name.split('/')
    return getattr(importlib.import_module(module + '.msg'), typename)

  def run(self):
    rospy.spin()