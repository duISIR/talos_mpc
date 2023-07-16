#! /usr/bin/env python3
import argparse

import rospy
import actionlib
from talos_msgs.msg import CmdTalosPoseAction, CmdTalosPoseGoal
import numpy as np
import optas

class CmdPoseClient(object):
    """docstring for CmdPoseClient."""

    def __init__(self, name, client, target_pos_Base, target_quat_Base, target_stiffness_spring_com, target_rest_length_spring_com, duration) -> None:
        # initialization message
        self._name = name
        self._target_pos_Base = target_pos_Base
        self._target_quat_Base = target_quat_Base
        self._target_stiffness_spring_com = target_stiffness_spring_com
        self._target_rest_length_spring_com = target_rest_length_spring_com
        self._duration = duration
        rospy.loginfo("%s: Initialized action client class.", self._name)
        # create actionlib client
        self._action_client = client
#        self._action_client = actionlib.SimpleActionClient('/chonk/cmd_pose', CmdChonkPoseAction)
        # wait until actionlib server starts
        rospy.loginfo("%s: Waiting for action server to start.", self._name)
        self._action_client.wait_for_server()
        rospy.loginfo("%s: Action server started, sending goal.", self._name)
        # creates goal and sends to the action server
        goal = CmdTalosPoseGoal()
        goal.poseBase.position.x = self._target_pos_Base[0]
        goal.poseBase.position.y = self._target_pos_Base[1]
        goal.poseBase.position.z = self._target_pos_Base[2]
        goal.poseBase.orientation.x = self._target_quat_Base[0]
        goal.poseBase.orientation.y = self._target_quat_Base[1]
        goal.poseBase.orientation.z = self._target_quat_Base[2]
        goal.poseBase.orientation.w = self._target_quat_Base[3]
        goal.stiffness_spring_com = self._target_stiffness_spring_com
        goal.rest_length_spring_com = self._target_rest_length_spring_com
        goal.duration = self._duration
        # sends the goal to the action server
        rospy.loginfo("%s: Send goal request to action server.", self._name)
        self._action_client.send_goal(
            goal,
#            done_cb=self.done_cb,
#            active_cb=self.active_cb,
            feedback_cb=self.feedback_cb
        )
        # wait for the server to finish the action
        self._action_client.wait_for_result()
        rospy.loginfo("%s: Got result from action server.", self._name)


    def done_cb(self, state, result):
        rospy.loginfo("%s: Action completed with result %r" % (self._name, result.reached_goal))
        rospy.signal_shutdown("Client request completed.")

    def active_cb(self):
        rospy.loginfo("%s: Goal went active!", self._name)

    def feedback_cb(self, feedback):
        rospy.loginfo("%s: %.1f%% to completion." % (self._name, feedback.progress))
        pass

if __name__ == '__main__':
    # parse arguments from terminal
    parser = argparse.ArgumentParser(description='Client node to command robot base and end-effector pose.')
    # parse CoM arguments
    CoM_position = [0, 0., 0.]
    parser.add_argument('--target_position_Base', nargs=3,
        help="Give target position of the robot in meters.",
        type=float, default=CoM_position,
        metavar=('POS_X', 'POS_Y', 'POS_Z')
    )
    Base_quaternion = optas.spatialmath.Quaternion.fromrpy([0, 0, 0]).getquat()
    parser.add_argument('--target_orientation_Base', nargs=4,
        help="Give target orientation as a quaternion.",
        type=float, default=[Base_quaternion[0], Base_quaternion[1], Base_quaternion[2], Base_quaternion[3]],
        metavar=('QUAT_X','QUAT_Y','QUAT_Z','QUAT_W')
    )

    parser.add_argument("--stiffness_spring_com", help="Give stiffness of spring at com.", type=float, default=1)
    parser.add_argument("--rest_length_spring_com", help="Give rest length of spring at com.", type=float, default=0.8)

    parser.add_argument("--duration", help="Give duration of motion in seconds.", type=float, default=10.0)
    args = vars(parser.parse_args())

    # Initialize node
    rospy.init_node('cmd_pose_client_MPC', anonymous=True)
    client = actionlib.SimpleActionClient('/talos/cmd_pose', CmdTalosPoseAction)

    # Initialize node class
    cmd_pose_client = CmdPoseClient('client', client,
        args['target_position_Base'],
        args['target_orientation_Base'],
        args['target_stiffness_spring_com'],
        args['target_rest_length_spring_com'],
        args['duration']
    )
    
