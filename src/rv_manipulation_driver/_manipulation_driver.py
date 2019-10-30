#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import rospy
import actionlib
import importlib
import tf
import yaml
import numpy as np
import quaternion
import xml.etree.ElementTree as ET

from ._control_switcher import ControlSwitcher
from ._action_proxy import ActionProxy
from ._manipulation_moveit_driver import ManipulationMoveItDriver

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist

from rv_msgs.msg import ManipulatorState
from rv_msgs.msg import MoveToPoseAction, MoveToPoseResult
from rv_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseResult
from rv_msgs.msg import ActuateGripperAction, ActuateGripperGoal, ActuateGripperResult
from rv_msgs.srv import GetNamesList, GetNamesListResponse, SetNamedPose, SetNamedPoseResponse
from rv_msgs.srv import GetRelativePose, GetRelativePoseResponse
from rv_msgs.srv import SetCartesianImpedance, SetCartesianImpedanceResponse

class ManipulationDriver(object):

  def __init__(self, moveit_commander=None):
    self.switcher = ControlSwitcher()

    # params
    self.controllers = rospy.get_param('~controllers', None)
    self.config_path = rospy.get_param('~config_path', os.path.join(os.environ.get('HOME'), '.ros/configs/manipulation_driver.yaml'))

    self.__load_config()

    if not moveit_commander:
        self.move_group = rospy.get_param('~move_group', None)

    if not self.controllers:
        rospy.logerr(
            'Unable to load controllers from rosparam server path: controllers'
        )
        sys.exit(1)

    if not self.move_group:
        rospy.logerr(
            'Unable to load move_group name from rosparam server path: move_group'
        )
        sys.exit(1)

    self.tf_listener = tf.TransformListener()

    rospy.Service('get_named_arm_poses', GetNamesList, self.get_named_poses_cb)
    rospy.Service('set_named_pose', SetNamedPose, self.set_named_pose_cb)

    rospy.Service('get_link_position', GetRelativePose, self.get_link_pose_cb)

    self.action_proxies = []
    self.publishers = []

    # for controller in self.controllers:
    #     if controller['type'] == 'publisher':
    #         self.publishers.append(
    #             self.create_publisher(
    #                 controller['controller'], controller['name'],
    #                 controller['maps'] if 'maps' in controller else None,
    #                 controller['topic_type']))

    #     if controller['type'] == 'action_server':
    #         self.action_proxies.append(
    #             self.create_action_server(
    #                 controller['controller'], controller['name'],
    #                 controller['maps'] if 'maps' in controller else None,
    #                 controller['topic_type']))


    self.velocity_subscriber = rospy.Subscriber('/cartesian/velocity', TwistStamped, self.velocity_cb)
    
    self.state_publisher = rospy.Publisher('/state', ManipulatorState, queue_size=1)
    
    self.moveit_commander = moveit_commander if moveit_commander else ManipulationMoveItDriver(
        group_name=self.move_group)

    self.pose_server = actionlib.SimpleActionServer(
        'cartesian/pose',
        MoveToPoseAction,
        execute_cb=self.pose_cb,
        auto_start=False)

    self.location_server = actionlib.SimpleActionServer(
        'cartesian/named_pose',
        MoveToNamedPoseAction,
        execute_cb=self.location_cb,
        auto_start=False)

    self.gripper_server = actionlib.SimpleActionServer(
        'gripper',
        ActuateGripperAction,
        execute_cb=self.gripper_cb,
        auto_start=False)

    self.pose_server.start()
    self.location_server.start()
    self.gripper_server.start()

    rospy.Service('home', Empty, self.home_cb)
    rospy.Service('recover', Empty, self.recover_cb)
    rospy.Service('stop', Empty, self.stop_cb)

    rospy.Service('cartesian/impedance', SetCartesianImpedance, self.set_cartesian_impedance_cb)
    # rospy.Service('force_torque_limits', SetForceTorqueImpedance, self.set_force_torque_cb)

  def create_publisher(self, controller_name, topic_in, topic_out,
                        topic_type_name):
    topic_type = self.__get_topic_type(topic_type_name)

    if topic_out:
        _publisher = rospy.Publisher(topic_out, topic_type, queue_size=1)

    def _callback(msg):
        if self.switcher.get_current_name() != controller_name:
            self.switcher.switch_controller(controller_name)

        if topic_out:
            _publisher.publish(msg)

    return rospy.Subscriber(topic_in, topic_type, _callback, queue_size=1)

  def velocity_cb(self, msg):
    pass

  def pose_cb(self, goal):
    if self.switcher.get_current_name() != 'position_joint_trajectory_controller':
        self.switcher.switch_controller('position_joint_trajectory_controller')

    self.moveit_commander.stop()
    self.moveit_commander.goto_pose(goal.goal_pose)
    self.pose_server.set_succeeded(MoveToPoseResult(result=0))

  def gripper_cb(self, goal):
    if goal.mode == ActuateGripperGoal.MODE_GRASP:
        result = self.moveit_commander.grasp(goal.width, goal.e_outer,
                                              goal.e_inner, goal.speed,
                                              goal.force)
        self.gripper_server.set_succeeded(
            ActuateGripperResult(result=result))

    elif goal.mode == ActuateGripperGoal.MODE_STATIC:
        result = self.moveit_commander.set_gripper(goal.width, goal.speed)
        self.gripper_server.set_succeeded(
            ActuateGripperResult(result=result))

    else:
        self.gripper_server.set_succeeded(ActuateGripperResult(result=1))

  def location_cb(self, goal):
    self.__move_to_named(goal.pose_name)

  def home_cb(self, req):
    self.__move_to_named('ready')
    return []

  def stop_cb(self, req):
    self.moveit_commander.stop()
    return []

  def recover_cb(self, req):
    return []

  def set_cartesian_impedance_cb(self, req):
    return True

  def get_named_poses_cb(self, req):
    poses = []
    if rospy.has_param('robot_description_semantic'):
        robot_description = rospy.get_param('robot_description_semantic')
        robot_description_struct = ET.fromstring(robot_description)

        for state in robot_description_struct.iter('group_state'):
            if state.attrib['group'] == self.move_group:
                poses.append(state.attrib['name'])

    return GetNamesListResponse(poses + self.named_poses.keys())

  def set_named_pose_cb(self, req):
    if req.pose_name in self.named_poses and not req.overwrite:
      rospy.logerr('Named pose already exists.')
      return SetNamedPoseResponse(success=False)
    
    self.named_poses[req.pose_name] = self.moveit_commander.get_joint_values()

    with open(self.config_path, 'w') as f:
      f.write(yaml.dump({ 'named_poses': self.named_poses }))
    
    return SetNamedPoseResponse(success=True)

  def get_link_pose_cb(self, req):
    response = GetRelativePoseResponse()
    response.relative_pose = self.get_link_pose(req.frame_reference, req.frame_target)
    return response

  def transform_velocity(self, msg, frame_target):
    if not msg.header.frame_id:
      return msg.twist

    (trans, rot) = self.tf_listener.lookupTransform(
      msg.header.frame_id, 
      frame_target,
      rospy.Time(0)
    )
    
    rot = quaternion.as_rotation_matrix(np.quaternion(rot[3], rot[0], rot[1], rot[2]))
    
    wJe =  np.concatenate(( np.concatenate((rot, np.zeros((3,3))), axis=1), np.concatenate((np.zeros((3,3)), rot), axis=1) ))
    
    eV = np.transpose(np.array([
      msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, 
      msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
    ], ndmin = 2))

    wV = np.matmul(wJe, eV)

    result = Twist()
    result.linear.x  = wV[0]
    result.linear.y  = wV[1]
    result.linear.z  = wV[2]
    result.angular.x = eV[3]
    result.angular.y = eV[4]
    result.angular.z = eV[5]

    return result

  def get_link_pose(self, frame_reference, frame_target):
    (trans, rot) = self.tf_listener.lookupTransform(
      frame_reference, 
      frame_target,
      rospy.Time(0)
    )

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_reference

    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]

    pose.pose.orientation.x = rot[0]
    pose.pose.orientation.y = rot[1]
    pose.pose.orientation.z = rot[2]
    pose.pose.orientation.w = rot[3]

    return pose

  def __move_to_named(self, named):
    if self.switcher.get_current_name(
    ) != 'position_joint_trajectory_controller':
        self.switcher.switch_controller(
            'position_joint_trajectory_controller')

    self.moveit_commander.stop()

    if named in self.named_poses:
      return self.moveit_commander.goto_joints(self.named_poses[named])
    else:
      return self.moveit_commander.goto_named_pose(named)

  def __get_topic_type(self, topic_type_name):
    module, typename = topic_type_name.split('/')
    return getattr(importlib.import_module(module + '.msg'), typename)

  def __load_config(self):
    if not os.path.exists(self.config_path):
      self.named_poses = {}
      return

    config = yaml.load(open(self.config_path))
    self.named_poses = config['named_poses']
    
  def run(self):
    rospy.spin()
