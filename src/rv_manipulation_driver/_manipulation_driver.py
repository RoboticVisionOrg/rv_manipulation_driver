#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import actionlib
import importlib
import tf
import xml.etree.ElementTree as ET

from ._control_switcher import ControlSwitcher
from ._action_proxy import ActionProxy
from ._manipulation_moveit_driver import ManipulationMoveItDriver

from std_srvs.srv import Empty

from rv_msgs.msg import MoveToPoseAction, MoveToPoseResult
from rv_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseResult
from rv_msgs.msg import ActuateGripperAction, ActuateGripperGoal, ActuateGripperResult
from rv_msgs.srv import GetNamesList, GetNamesListResponse
from rv_msgs.srv import GetRelativePose, GetRelativePoseResponse


class ManipulationDriver(object):

    def __init__(self, moveit_commander=None):
        self.switcher = ControlSwitcher()

        # params
        self.controllers = rospy.get_param('~controllers', None)

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
        rospy.Service('get_link_position', GetRelativePose, self.get_link_pose_cb)

        self.action_proxies = []
        self.publishers = []

        for controller in self.controllers:
            if controller['type'] == 'publisher':
                self.publishers.append(
                    self.create_publisher(
                        controller['controller'], controller['name'],
                        controller['maps'] if 'maps' in controller else None,
                        controller['topic_type']))

            if controller['type'] == 'action_server':
                self.action_proxies.append(
                    self.create_action_server(
                        controller['controller'], controller['name'],
                        controller['maps'] if 'maps' in controller else None,
                        controller['topic_type']))

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

    def pose_cb(self, goal):
        if self.switcher.get_current_name(
        ) != 'position_joint_trajectory_controller':
            self.switcher.switch_controller(
                'position_joint_trajectory_controller')

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

    def get_named_poses_cb(self, req):
        poses = []
        if rospy.has_param('robot_description_semantic'):
            robot_description = rospy.get_param('robot_description_semantic')
            robot_description_struct = ET.fromstring(robot_description)

            for state in robot_description_struct.iter('group_state'):
                if state.attrib['group'] == self.move_group:
                    poses.append(state.attrib['name'])

        return GetNamesListResponse(poses)

    def get_link_pose_cb(self, req):
        (trans,
         rot) = self.tf_listener.lookupTransform(req.base_frame, req.link_name,
                                                 rospy.Time(0))
        response = GetRelativePoseResponse()

        response.relative_pose.header.stamp = rospy.Time.now()
        response.relative_pose.header.frame_id = req.base_frame

        response.relative_pose.pose.position.x = trans[0]
        response.relative_pose.pose.position.y = trans[1]
        response.relative_pose.pose.position.z = trans[2]

        response.relative_pose.pose.orientation.x = rot[0]
        response.relative_pose.pose.orientation.y = rot[1]
        response.relative_pose.pose.orientation.z = rot[2]
        response.relative_pose.pose.orientation.w = rot[3]

        return response

    def __move_to_named(self, named):
        if self.switcher.get_current_name(
        ) != 'position_joint_trajectory_controller':
            self.switcher.switch_controller(
                'position_joint_trajectory_controller')

        self.moveit_commander.stop()
        return self.moveit_commander.goto_named_pose(named)

    def __get_topic_type(self, topic_type_name):
        module, typename = topic_type_name.split('/')
        return getattr(importlib.import_module(module + '.msg'), typename)

    def run(self):
        rospy.spin()
