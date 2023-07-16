#! /usr/bin/env python3

# Copyright (C) 2022 Statistical Machine Learning and Motor Control Group (SLMC)
# Authors: Joao Moura (maintainer)
# email: joao.moura@ed.ac.uk

# This file is part of iiwa_pushing package.

# iiwa_pushing is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# iiwa_pushing is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
import sys
import numpy as np
np.set_printoptions(suppress=True)

import rospy
import actionlib
import optas
import casadi

from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from talos_msgs.msg import CmdTalosPoseAction, CmdTalosPoseFeedback, CmdTalosPoseResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from urdf_parser_py.urdf import URDF

# For mux controller name
from std_msgs.msg import String
# service for selecting the controller
from topic_tools.srv import MuxSelect
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

from talos_dynamics import TALOS

class CmdPoseActionServer(object):
    """docstring for CmdPoseActionServer."""

    def __init__(self, name):
        # initialization message
        self._name = name
        rospy.loginfo("%s: Initializing class", self._name)
        ## get parameters:
        ## --------------------------------------
        # workspace limit boundaries
        self._x_min = rospy.get_param('~x_min', -10000)
        self._x_max = rospy.get_param('~x_max',  10000)
        self._y_min = rospy.get_param('~y_min', -10000)
        self._y_max = rospy.get_param('~y_max',  10000)
        self._z_min = rospy.get_param('~z_min', -10000)
        self._z_max = rospy.get_param('~z_max',  10000)
        self._pos_min = np.asarray([self._x_min, self._y_min, self._z_min])
        self._pos_max = np.asarray([self._x_max, self._y_max, self._z_max])
        # robot name
        # donkey_base frame
        self._link_base = rospy.get_param('~link_base', 'link_base')
        # end-effector frame
        self._link_ee_right_arm = rospy.get_param('~link_ee_right_arm', 'link_ee_right_arm')
        self._link_ee_left_arm = rospy.get_param('~link_ee_left_arm', 'link_ee_left_arm')
        self._link_ee_right_leg = rospy.get_param('~link_ee_right_leg', 'link_ee_right_leg')
        self._link_ee_left_leg = rospy.get_param('~link_ee_left_leg', 'link_ee_left_leg')
        self._link_head = rospy.get_param('~link_head', 'link_head')
        self.talos = TALOS('TALOS DYNAMICS')
        # control frequency
        self._freq = rospy.get_param('~freq', 50)
        # publishing command node name
        self._pub_cmd_topic_name = rospy.get_param('~cmd_topic_name', '/command')
        # load robot_description
        param_robot_description = '~/robot_description_talos'
        if rospy.has_param(param_robot_description):
            self._robot_description = rospy.get_param(param_robot_description)
            self._urdf = URDF.from_parameter_server(param_robot_description)
        else:
            rospy.logerr("%s: Param %s is unavailable!" % (self._name, param_robot_description))
            rospy.signal_shutdown('Incorrect parameter name.')
        self.joint_names = [jnt.name for jnt in self._urdf.joints if (jnt.type != 'fixed')]
        # print(self.joint_names)
        self.ndof_base = 6
        self.ndof_joints = len(self.joint_names) - self.ndof_base
        self.ndof = self.ndof_base + self.ndof_joints
        ### ---------------------------------------------------------
        # initialize variables for planner
        self.q_curr = np.zeros(self.ndof)
        self.q_curr_joint = np.zeros(self.ndof_joints)
        self.q_curr_joint_fb = np.zeros(self.ndof_joints+2)
        self.q_curr_base = np.zeros(self.ndof_base)
        self.dq_curr = np.zeros(self.ndof)
        self.dq_curr_joint = np.zeros(self.ndof_joints)
        self.dq_curr_joint_fb = np.zeros(self.ndof_joints+2)
        self.dq_curr_base = np.zeros(self.ndof_base)
        self.q_joints_output = np.zeros(self.ndof+2)
        self.ddq_joints_output = np.zeros(self.ndof+2)
        self.joint_names_position = []
        self.joint_names_base = ['base_joint_1', 'base_joint_2', 'base_joint_3', 'base_joint_4', 'base_joint_5', 'base_joint_6']
        self.donkey_R = np.zeros((3, 3))
        self.donkey_position = np.zeros(3)
        self.donkey_velocity = np.zeros(3)
        self.donkey_angular_velocity = np.zeros(3)
        ### optas
        ### ---------------------------------------------------------
        # set up whole-body MPC
        wholebodyMPC_LIMITS = optas.RobotModel(urdf_string=self._robot_description, time_derivs=[0, 1], param_joints=[], name='talos_wholebodyMPC_LIMITS')
        self.wholebodyMPC = optas.RobotModel(
            urdf_string=self._robot_description, time_derivs=[0],
            param_joints=['base_joint_1', 'base_joint_2', 'base_joint_3', 'base_joint_4', 'base_joint_5', 'base_joint_6',
                          'torso_1_joint', 'torso_2_joint', 'head_1_joint', 'head_2_joint',
                          'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint',
                          'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint',
                          'leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint',
                          'leg_right_1_joint', 'leg_right_2_joint', 'leg_right_3_joint', 'leg_right_4_joint', 'leg_right_5_joint', 'leg_right_6_joint'],
            name='talos_wholebodyMPC' )
        lower, upper = wholebodyMPC_LIMITS.get_limits(time_deriv=0)
        dlower, dupper = wholebodyMPC_LIMITS.get_limits(time_deriv=1)
        self.wholebodyMPC_name = self.wholebodyMPC.get_name()
        self.dt_MPC = 0.1 # time step
        self.T_MPC = 7 # T is number of time steps
        # self.duration_MPC = float(self.T_MPC-1)*self.dt_MPC
        # nominal robot configuration
        self.wholebodyMPC_opt_idx = self.wholebodyMPC.optimized_joint_indexes
        self.wholebodyMPC_param_idx = self.wholebodyMPC.parameter_joint_indexes
        # set up optimization builder.
        builder_wholebodyMPC = optas.OptimizationBuilder(T=1, robots=[self.wholebodyMPC])
        builder_wholebodyMPC._decision_variables = optas.sx_container.SXContainer()
        builder_wholebodyMPC._parameters = optas.sx_container.SXContainer()
        builder_wholebodyMPC._lin_eq_constraints = optas.sx_container.SXContainer()
        builder_wholebodyMPC._lin_ineq_constraints = optas.sx_container.SXContainer()
        builder_wholebodyMPC._ineq_constraints = optas.sx_container.SXContainer()
        builder_wholebodyMPC._eq_constraints = optas.sx_container.SXContainer()
        # get robot state variables, get velocity state variables
        Q = builder_wholebodyMPC.add_decision_variables('Q', self.ndof, self.T_MPC)

        t = builder_wholebodyMPC.add_parameter('t', self.T_MPC)  # time
        # t_periodic = builder_wholebodyMPC.add_parameter('t_periodic', self.T_MPC * 2-1)  # time
        duration_MPC = builder_wholebodyMPC.add_parameter('duration_MPC', 1)  # step period

        self.n = self.T_MPC -1 # N in Bezier curve
        # Add parameters
        init_position_MPC = builder_wholebodyMPC.add_parameter('init_position_MPC', self.ndof)  # initial robot position
        init_velocity_MPC = builder_wholebodyMPC.add_parameter('init_velocity_MPC', self.ndof)  # initial robot velocity
        ###################################################################################
        # get end-effector pose as parameters
        pos_base_goal = builder_wholebodyMPC.add_parameter('pos_base_goal', 3, self.T_MPC)
        quat_base_goal = builder_wholebodyMPC.add_parameter('quat_base_goal', 4, self.T_MPC)
        pos_com_goal_z = builder_wholebodyMPC.add_parameter('pos_com_goal_z', 1, self.T_MPC)
        #####################################################################################
        # functions of right and left arm positions
        self.pos_fnc_Right_arm = self.wholebodyMPC.get_global_link_position_function(link=self._link_ee_right_arm)
        self.pos_fnc_Left_arm = self.wholebodyMPC.get_global_link_position_function(link=self._link_ee_left_arm)
        self.pos_Jac_fnc_Right_arm = self.wholebodyMPC.get_global_link_linear_jacobian_function(link=self._link_ee_right_arm)
        self.pos_Jac_fnc_Left_arm = self.wholebodyMPC.get_global_link_linear_jacobian_function(link=self._link_ee_left_arm)
        # functions of right and left leg positions
        self.pos_fnc_Right_leg = self.wholebodyMPC.get_global_link_position_function(link=self._link_ee_right_leg)
        self.pos_fnc_Left_leg = self.wholebodyMPC.get_global_link_position_function(link=self._link_ee_left_leg)
        self.pos_Jac_fnc_Right_leg = self.wholebodyMPC.get_global_link_linear_jacobian_function(link=self._link_ee_right_leg)
        self.pos_Jac_fnc_Left_leg = self.wholebodyMPC.get_global_link_linear_jacobian_function(link=self._link_ee_left_leg)
        # quaternion functions of two arm end effectors
        self.ori_fnc_Right_arm = self.wholebodyMPC.get_global_link_quaternion_function(link=self._link_ee_right_arm)
        self.ori_fnc_Left_arm = self.wholebodyMPC.get_global_link_quaternion_function(link=self._link_ee_left_arm)
        self.rotation_fnc_Right_arm = self.wholebodyMPC.get_global_link_rotation_function(link=self._link_ee_right_arm)
        self.rotation_fnc_Left_arm = self.wholebodyMPC.get_global_link_rotation_function(link=self._link_ee_left_arm)
        # quaternion functions of two leg end effectors
        self.ori_fnc_Right_leg = self.wholebodyMPC.get_global_link_quaternion_function(link=self._link_ee_right_leg)
        self.ori_fnc_Left_leg = self.wholebodyMPC.get_global_link_quaternion_function(link=self._link_ee_left_leg)
        self.rotation_fnc_Right_leg = self.wholebodyMPC.get_global_link_rotation_function(link=self._link_ee_right_leg)
        self.rotation_fnc_Left_leg = self.wholebodyMPC.get_global_link_rotation_function(link=self._link_ee_left_leg)
        # define four-limb end-effector functions w.r.t. the base
        self.pos_fnc_Right_leg_wrt_base = self.wholebodyMPC.get_link_position_function(link=self._link_ee_right_leg, base_link=floatingbase_link) 
        self.ori_fnc_Right_leg_wrt_base = self.wholebodyMPC.get_link_rotation_function(link=self._link_ee_right_leg, base_link=floatingbase_link)
        self.pos_fnc_Left_leg_wrt_base = self.wholebodyMPC.get_link_position_function(link=self._link_ee_left_leg, base_link=floatingbase_link) 
        self.ori_fnc_Left_leg_wrt_base = self.wholebodyMPC.get_link_rotation_function(link=self._link_ee_left_leg, base_link=floatingbase_link)
        self.pos_fnc_Right_arm_wrt_base = self.wholebodyMPC.get_link_position_function(link=self._link_ee_right_arm, base_link=floatingbase_link) 
        self.ori_fnc_Right_arm_wrt_base = self.wholebodyMPC.get_link_rotation_function(link=self._link_ee_right_arm, base_link=floatingbase_link)
        self.pos_fnc_Left_arm_wrt_base = self.wholebodyMPC.get_link_position_function(link=self._link_ee_left_arm, base_link=floatingbase_link) 
        self.ori_fnc_Left_arm_wrt_base = self.wholebodyMPC.get_link_rotation_function(link=self._link_ee_left_arm, base_link=floatingbase_link)
        #####################################################################################
        floatingbase_link = self.talos.link[1];
        self.link_pos = []; self.link_rot = []; self.link_quat = []; 
        self.link_pos.append(np.zeros(3)); self.link_rot.append(np.identity(3)); self.link_quat.append(np.array([0,0,0,1]));
        self.link_pos_wrt_base = []; self.link_rot_wrt_base = [];
        self.link_pos_wrt_base.append(np.zeros(3)); self.link_rot_wrt_base.append(np.identity(3));
        # self.link_Jac_pos = []; self.link_Jac_ori = []; self.link_Jac = [];
        # self.link_Jac_pos.append(np.zeros((3, self.ndof))); self.link_Jac_ori.append(np.zeros((3, self.ndof))); self.link_Jac.append(np.zeros((6, self.ndof)));
        self.link_Jac_wrt_base = [];
        self.link_Jac_wrt_base.append(np.zeros((6, self.ndof)));

        for i in range(1, self.talos.link_num+1):
            self.link_pos.append(self.wholebodyMPC.get_global_link_position_function(link=self.talos.link[i])) # get the function that computes the position of a link in global frame.
            self.link_rot.append(self.wholebodyMPC.get_global_link_rotation_function(link=self.talos.link[i]))  # get the function that computes the rotation matrix of a link in global frame.
            self.link_quat.append(self.wholebodyMPC.get_global_link_quaternion_function(link=self.talos.link[i]))  # get the function that computes the quaternion of a link in global frame.
            self.link_pos_wrt_base.append(self.wholebodyMPC.get_link_position_function(link=self.talos.link[i], base_link=floatingbase_link)) # get the function that computes the position of a link in base frame. 
            self.link_rot_wrt_base.append(self.wholebodyMPC.get_link_rotation_function(link=self.talos.link[i], base_link=floatingbase_link)) # get the function that computes the rotation matrix of a link in base frame. 
            # self.link_Jac_pos.append(self.wholebodyMPC.get_global_link_linear_jacobian_function(link=self.talos.link[i]))
            # self.link_Jac_ori.append(self.wholebodyMPC.get_global_link_angular_geometric_jacobian_function(link=self.talos.link[i]))
            # self.link_Jac.append(self.wholebodyMPC.get_global_link_geometric_jacobian_function(link=self.talos.link[i]))
            self.link_Jac_wrt_base.append(self.wholebodyMPC.get_link_geometric_jacobian_function(link=self.talos.link[i], base_link=floatingbase_link))

        ##########################################################################################################################################################################
        # define q function depending on P
        # self.pos_com = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        # self.l_com = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        # self.k_com = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        # self.h_com = optas.casadi.SX(np.zeros((6, self.T_MPC)))

        pos_com_wrt_base_goal = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        for i in range(self.T_MPC):
            quaternion = optas.spatialmath.Quaternion(quat_base_goal[0, i], quat_base_goal[1, i], quat_base_goal[2, i], quat_base_goal[3, i])
            pos_com_wrt_base_goal[2, i] = (pos_com_goal_z[i] - pos_base_goal[2, i])/float(quaternion.getrotm()[2, 2])
        ##########################################################################################################################################################################
        q_var_MPC = optas.casadi.SX(np.zeros((self.ndof, self.T_MPC)))
        for i in range(self.T_MPC):
            for j in range(self.T_MPC):
                q_var_MPC[:, i] += self.BC(self.n, j) * ((t[i] - t[0])/duration_MPC)**j * (1-((t[i] - t[0])/duration_MPC))**(self.n-j) * Q[:, j]

        self.rot_base = []
        for i in range(self.T_MPC):
            self.rot_base.append(optas.casadi.SX(np.zeros((3,3))))
        self.quat_base = optas.casadi.SX(np.zeros((4, self.T_MPC)))        
        self.pos_base = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        self.pos_com_wrt_base = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        for i in range(self.T_MPC):
            self.pos_base[:, i] = self.link_pos[1](q_var_MPC[:, i])
            self.quat_base[:, i] = self.link_quat[1](q_var_MPC[:, i])
            self.rot_base[i] = self.link_rot[1](q_var_MPC[:, i])
            for j in range(1, self.talos.link_num+1):
                self.pos_com_wrt_base[:, i] += (1/self.talos.m_G) *(self.talos.m[j] * self.link_pos_wrt_base[j](q_var_MPC[:, i]))

        self.pos_com_wrt_base_by_larm = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        self.pos_com_wrt_base_by_rarm = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        self.pos_com_wrt_base_by_lleg = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        self.pos_com_wrt_base_by_rleg = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        for i in range(self.T_MPC):
            for j in range(1, self.talos.link_num+1):
                if(j>=6 and j<=12): # left arm bodies
                    self.pos_com_wrt_base_by_larm[:, i] += (1/self.talos.m_G) *(self.talos.m[j] * self.link_pos_wrt_base[j](q_var_MPC[:, i]))
                if(j>=13 and j<=19): # right arm bodies
                    self.pos_com_wrt_base_by_rarm[:, i] += (1/self.talos.m_G) *(self.talos.m[j] * self.link_pos_wrt_base[j](q_var_MPC[:, i]))
                if(j>=20 and j<=25): # left leg bodies
                    self.pos_com_wrt_base_by_lleg[:, i] += (1/self.talos.m_G) *(self.talos.m[j] * self.link_pos_wrt_base[j](q_var_MPC[:, i]))
                if(j>=26 and j<=31): # left leg bodies
                    self.pos_com_wrt_base_by_rleg[:, i] += (1/self.talos.m_G) *(self.talos.m[j] * self.link_pos_wrt_base[j](q_var_MPC[:, i]))
        ##########################################################################################################################################################################
        self.I_b_C = []; self.Omega = []; self.f_Omega = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        for i in range(self.T_MPC):
            self.I_b_C.append(optas.casadi.SX(np.zeros((6,6))))
            self.Omega.append(optas.casadi.SX(np.zeros((3,3))))
        for i in range(self.T_MPC):
            for j in range(1, self.talos.link_num+1):
                i_X_b = optas.casadi.SX(np.zeros((6,6)))
                i_X_b = self.i_X_b(self.link_pos_wrt_base[j](q_var_MPC[:, i]), self.link_rot_wrt_base[j](q_var_MPC[:, i]))
                self.I_b_C[i] += i_X_b.T @ self.talos.I[j] @ i_X_b
            self.Omega[i] = self.I_b_C[i][0:3, 0:3] + optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) @ optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) * self.talos.m_G;
            self.f_Omega[0, i] = self.Omega[i][0, 1]
            self.f_Omega[1, i] = self.Omega[i][0, 2]
            self.f_Omega[2, i] = self.Omega[i][1, 2]

        self.I_b_C_by_larm = []; self.Omega_by_larm = []; self.f_Omega_by_larm = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        self.I_b_C_by_rarm = []; self.Omega_by_rarm = []; self.f_Omega_by_rarm = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        self.I_b_C_by_lleg = []; self.Omega_by_lleg = []; self.f_Omega_by_lleg = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        self.I_b_C_by_rleg = []; self.Omega_by_rleg = []; self.f_Omega_by_rleg = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        for i in range(self.T_MPC):
            self.I_b_C_by_larm.append(optas.casadi.SX(np.zeros((6,6)))); self.Omega_by_larm.append(optas.casadi.SX(np.zeros((3,3)))); 
            self.I_b_C_by_rarm.append(optas.casadi.SX(np.zeros((6,6)))); self.Omega_by_rarm.append(optas.casadi.SX(np.zeros((3,3)))); 
            self.I_b_C_by_lleg.append(optas.casadi.SX(np.zeros((6,6)))); self.Omega_by_lleg.append(optas.casadi.SX(np.zeros((3,3)))); 
            self.I_b_C_by_rleg.append(optas.casadi.SX(np.zeros((6,6)))); self.Omega_by_rleg.append(optas.casadi.SX(np.zeros((3,3)))); 
        for i in range(self.T_MPC):
            for j in range(1, self.talos.link_num+1):
                if(i>=6 and i<=12): # left arm bodies
                    i_X_b = optas.casadi.SX(np.zeros((6,6)))
                    i_X_b = self.i_X_b(self.link_pos_wrt_base[j](q_var_MPC[:, i]), self.link_rot_wrt_base[j](q_var_MPC[:, i]))
                    self.I_b_C_by_larm[i] += i_X_b.T @ self.talos.I[j] @ i_X_b
                    self.Omega_by_larm[i] = self.I_b_C_by_larm[i][0:3, 0:3] + optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) @ optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) * self.talos.m_larm; 
                    self.f_Omega_by_larm[0, i] = self.Omega_by_larm[i][0, 1]
                    self.f_Omega_by_larm[1, i] = self.Omega_by_larm[i][0, 2]
                    self.f_Omega_by_larm[2, i] = self.Omega_by_larm[i][1, 2]   
                if(i>=13 and i<=19): # right arm bodies
                    i_X_b = optas.casadi.SX(np.zeros((6,6)))
                    i_X_b = self.i_X_b(self.link_pos_wrt_base[j](q_var_MPC[:, i]), self.link_rot_wrt_base[j](q_var_MPC[:, i]))
                    self.I_b_C_by_rarm[i] += i_X_b.T @ self.talos.I[j] @ i_X_b
                    self.Omega_by_rarm[i] = self.I_b_C_by_rarm[i][0:3, 0:3] + optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) @ optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) * self.talos.m_rarm; 
                    self.f_Omega_by_rarm[0, i] = self.Omega_by_rarm[i][0, 1]
                    self.f_Omega_by_rarm[1, i] = self.Omega_by_rarm[i][0, 2]
                    self.f_Omega_by_rarm[2, i] = self.Omega_by_rarm[i][1, 2]  
                if(i>=20 and i<=25): # left leg bodies
                    i_X_b = optas.casadi.SX(np.zeros((6,6)))
                    i_X_b = self.i_X_b(self.link_pos_wrt_base[j](q_var_MPC[:, i]), self.link_rot_wrt_base[j](q_var_MPC[:, i]))
                    self.I_b_C_by_lleg[i] += i_X_b.T @ self.talos.I[j] @ i_X_b
                    self.Omega_by_lleg[i] = self.I_b_C_by_lleg[i][0:3, 0:3] + optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) @ optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) * self.talos.m_lleg; 
                    self.f_Omega_by_lleg[0, i] = self.Omega_by_lleg[i][0, 1]
                    self.f_Omega_by_lleg[1, i] = self.Omega_by_lleg[i][0, 2]
                    self.f_Omega_by_lleg[2, i] = self.Omega_by_lleg[i][1, 2]  
                if(i>=26 and i<=31): # left leg bodies
                    i_X_b = optas.casadi.SX(np.zeros((6,6)))
                    i_X_b = self.i_X_b(self.link_pos_wrt_base[j](q_var_MPC[:, i]), self.link_rot_wrt_base[j](q_var_MPC[:, i]))
                    self.I_b_C_by_rleg[i] += i_X_b.T @ self.talos.I[j] @ i_X_b
                    self.Omega_by_rleg[i] = self.I_b_C_by_rleg[i][0:3, 0:3] + optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) @ optas.spatialmath.skew(self.pos_com_wrt_base[:, i]) * self.talos.m_rleg; 
                    self.f_Omega_by_rleg[0, i] = self.Omega_by_rleg[i][0, 1]
                    self.f_Omega_by_rleg[1, i] = self.Omega_by_rleg[i][0, 2]
                    self.f_Omega_by_rleg[2, i] = self.Omega_by_rleg[i][1, 2]  
        ##########################################################################################################################################################################
        for i in range(self.T_MPC):
            builder_wholebodyMPC.add_bound_inequality_constraint('control_point_' + str(i) + '_bound', lhs=lower, mid=Q[:, i], rhs=upper)
            # optimization cost: close to base target
            builder_wholebodyMPC.add_cost_term('pos_base' + str(i), optas.sumsqr(self.pos_base[:, i] - pos_base_goal[:, i]))
            builder_wholebodyMPC.add_cost_term('quat_base' + str(i), optas.sumsqr(self.quat_base[:, i] - quat_base_goal[:, i]))
            # optimization cost: close to relative com target
            builder_wholebodyMPC.add_cost_term('pos_com_wrt_base' + str(i), optas.sumsqr(self.pos_com_wrt_base[:, i] - pos_com_wrt_base_goal[:, i]))
            builder_wholebodyMPC.add_cost_term('rot_com_wrt_base' + str(i), optas.sumsqr(self.f_Omega[:, i]))
        ##########################################################################################################################################################################
        for i in range(self.T_MPC):
            builder_wholebodyMPC.add_leq_inequality_constraint('diagonal_leg_move_opposite_pos_x' + str(i), lhs=(self.pos_com_wrt_base_by_larm[:, i] + self.pos_com_wrt_base_by_rleg[:, i] - self.pos_com_wrt_base[:, i])[0] * (self.pos_com_wrt_base_by_rarm[:, i] + self.pos_com_wrt_base_by_lleg[:, i] - self.pos_com_wrt_base[:, i])[0], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('diagonal_leg_move_opposite_pos_y' + str(i), lhs=(self.pos_com_wrt_base_by_larm[:, i] + self.pos_com_wrt_base_by_rleg[:, i] - self.pos_com_wrt_base[:, i])[1] * (self.pos_com_wrt_base_by_rarm[:, i] + self.pos_com_wrt_base_by_lleg[:, i] - self.pos_com_wrt_base[:, i])[1], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_leg_move_opposite_pos_x' + str(i), lhs=(self.pos_com_wrt_base_by_rleg[:, i] - self.pos_com_wrt_base[:, i])[0] * (self.pos_com_wrt_base_by_lleg[:, i] - self.pos_com_wrt_base[:, i])[0], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_leg_move_opposite_pos_y' + str(i), lhs=(self.pos_com_wrt_base_by_rleg[:, i] - self.pos_com_wrt_base[:, i])[1] * (self.pos_com_wrt_base_by_lleg[:, i] - self.pos_com_wrt_base[:, i])[1], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_leg_move_opposite_pos_x' + str(i), lhs=(self.pos_com_wrt_base_by_rarm[:, i] - self.pos_com_wrt_base[:, i])[0] * (self.pos_com_wrt_base_by_larm[:, i] - self.pos_com_wrt_base[:, i])[0], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_arm_move_opposite_pos_y' + str(i), lhs=(self.pos_com_wrt_base_by_rarm[:, i] - self.pos_com_wrt_base[:, i])[1] * (self.pos_com_wrt_base_by_larm[:, i] - self.pos_com_wrt_base[:, i])[1], rhs=0)
        ##########################################################################################################################################################################
        for i in range(self.T_MPC):
            builder_wholebodyMPC.add_leq_inequality_constraint('diagonal_leg_move_opposite_ori_x' + str(i), lhs=(self.f_Omega_by_larm[:, i] + self.f_Omega_by_rleg[:, i])[0] * (self.f_Omega_by_rarm[:, i] + self.f_Omega_by_lleg[:, i])[0], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('diagonal_leg_move_opposite_ori_y' + str(i), lhs=(self.f_Omega_by_larm[:, i] + self.f_Omega_by_rleg[:, i])[1] * (self.f_Omega_by_rarm[:, i] + self.f_Omega_by_lleg[:, i])[1], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('diagonal_leg_move_opposite_ori_z' + str(i), lhs=(self.f_Omega_by_larm[:, i] + self.f_Omega_by_rleg[:, i])[2] * (self.f_Omega_by_rarm[:, i] + self.f_Omega_by_lleg[:, i])[2], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_leg_move_opposite_ori_x' + str(i), lhs=(self.f_Omega_by_rleg[:, i])[0] * (self.f_Omega_by_lleg[:, i])[0], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_leg_move_opposite_ori_y' + str(i), lhs=(self.f_Omega_by_rleg[:, i])[1] * (self.f_Omega_by_lleg[:, i])[1], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_leg_move_opposite_ori_z' + str(i), lhs=(self.f_Omega_by_rleg[:, i])[2] * (self.f_Omega_by_lleg[:, i])[2], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_leg_move_opposite_ori_x' + str(i), lhs=(self.f_Omega_by_rarm[:, i])[0] * (self.f_Omega_by_larm[:, i])[0], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_arm_move_opposite_ori_y' + str(i), lhs=(self.f_Omega_by_rarm[:, i])[1] * (self.f_Omega_by_larm[:, i])[1], rhs=0)
            builder_wholebodyMPC.add_leq_inequality_constraint('two_arm_move_opposite_ori_z' + str(i), lhs=(self.f_Omega_by_rarm[:, i])[2] * (self.f_Omega_by_larm[:, i])[2], rhs=0)
        ##########################################################################################################################################################################
        for i in range(self.T_MPC):
            builder_wholebodyMPC.add_cost_term('twoarm_miniscope' + str(i), 0.1 * optas.sumsqr(q_var_MPC[10, i]+q_var_MPC[17, i]))
            builder_wholebodyMPC.add_cost_term('arm_joint_miniscope' + str(i), 0.001 * optas.sumsqr(q_var_MPC[10:24, i]))
            if(i<(self.T_MPC -1)):
                builder_wholebodyMPC.add_cost_term('distance' + str(i), 0.05 * optas.sumsqr(Q[:, i+1] - Q[:, i]))
        ##########################################################################################################################################################################
        p_palm_wrt_base = optas.casadi.SX(np.zeros((3, self.T_MPC * 2-1)))
        p_palm_lleg_wrt_base = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        p_palm_rleg_wrt_base = optas.casadi.SX(np.zeros((3, self.T_MPC)))
        theta_palm_wrt_base = optas.casadi.SX(np.zeros((4, self.T_MPC * 2-1)))
        theta_palm_lleg_wrt_base = optas.casadi.SX(np.zeros((4, self.T_MPC)))
        theta_palm_rleg_wrt_base = optas.casadi.SX(np.zeros((4, self.T_MPC)))

        P = optas.casadi.SX(np.zeros((3, self.T_MPC * 2)))
        THETA = optas.casadi.SX(np.zeros((4, self.T_MPC * 2)))

        # self.time_linspace = np.linspace(0., duration_MPC * 2, self.T_MPC * 2 - 1)
        # self.timeby2T = np.asarray(self.time_linspace)/(duration_MPC*2)

        # for i in range(self.T_MPC * 2 -1):
        #     for j in range(self.T_MPC * 2):
        #         p_palm_wrt_base[:, i] += self.BC(self.n, j) * self.timeby2T[i]**j * (1-self.timeby2T[i])**(self.n-j) * P[:, j]
        #         theta_palm_wrt_base[:, i] += self.BC(self.n, j) * self.timeby2T[i]**j * (1-self.timeby2T[i])**(self.n-j) * THETA[:, j]

        for i in range(self.T_MPC):
            for j in range(self.T_MPC*2):
                p_palm_lleg_wrt_base[:, i] +=self.BC(self.n, j) * (casadi.mod(t[i], duration_MPC*2)/(duration_MPC*2))**j * (1-(casadi.mod(t[i], duration_MPC*2)/(duration_MPC*2)))**(self.n-j) * P[:, j]
                theta_palm_lleg_wrt_base[:, i] +=self.BC(self.n, j) * (casadi.mod(t[i], duration_MPC*2)/(duration_MPC*2))**j * (1-(casadi.mod(t[i], duration_MPC*2)/(duration_MPC*2)))**(self.n-j) * THETA[:, j]
            builder_wholebodyMPC.add_equality_constraint(name, lhs=self.pos_fnc_Left_leg_wrt_base(q_var_MPC[:, i]), rhs=p_palm_lleg_wrt_base[:, i])
            builder_wholebodyMPC.add_equality_constraint(name, lhs=self.ori_fnc_Left_leg_wrt_base(q_var_MPC[:, i]), rhs=p_palm_lleg_wrt_base[:, i])

        for i in range(self.T_MPC-1, self.T_MPC*2-1):
            for j in range(self.T_MPC*2):
                p_palm_rleg_wrt_base[:, i] +=self.BC(self.n, j) * (casadi.mod(t[i]+duration_MPC, duration_MPC*2)/(duration_MPC*2))**j * (1-(casadi.mod(t[i]+duration_MPC, duration_MPC*2)/(duration_MPC*2)))**(self.n-j) * P[:, j]
                theta_palm_rleg_wrt_base[:, i] +=self.BC(self.n, j) * (casadi.mod(t[i]+duration_MPC, duration_MPC*2)/(duration_MPC*2))**j * (1-(casadi.mod(t[i]+duration_MPC, duration_MPC*2)/(duration_MPC*2)))**(self.n-j) * THETA[:, j]
            builder_wholebodyMPC.add_equality_constraint(name, lhs=self.pos_fnc_Right_leg_wrt_base(q_var_MPC[:, i]), rhs=p_palm_rleg_wrt_base[:, i])
            builder_wholebodyMPC.add_equality_constraint(name, lhs=self.ori_fnc_Right_leg_wrt_base(q_var_MPC[:, i]), rhs=p_palm_rleg_wrt_base[:, i])
        
        for i in range(self.T_MPC * 2 -1):
            for j in range(self.T_MPC * 2):
                p_palm_wrt_base[:, i] +=self.BC(self.n, j) * (casadi.mod(t[i], duration_MPC)/(duration_MPC*2))**j * (1-(casadi.mod(t[i], duration_MPC)/(duration_MPC*2)))**(self.n-j) * P[:, j]
                theta_palm_wrt_base[:, i] +=self.BC(self.n, j) * (casadi.mod(t[i], duration_MPC)/(duration_MPC*2))**j * (1-(casadi.mod(t[i], duration_MPC)/(duration_MPC*2)))**(self.n-j) * THETA[:, j]          
            builder_wholebodyMPC.add_equality_constraint(name, lhs=self.rot_base[0] @  p_palm_wrt_base[:, 0] - self.rot_base[i] @ p_palm_wrt_base[:, i], rhs=self.pos_base[i] - self.pos_base[0])
            builder_wholebodyMPC.add_equality_constraint(name, lhs=self.qaQb(self.quat_base[0], theta_palm_wrt_base[:, 0]), rhs=self.qaQb(self.quat_base[i], theta_palm_wrt_base[:, i]))
            builder_wholebodyMPC.add_equality_constraint(name, lhs=self.qaQb(self.quat_base[i], theta_palm_wrt_base[:, i])[0:2], rhs=np.zeros(2))
        ##########################################################################################################################################################################
        dq_var_MPC = optas.casadi.SX(np.zeros((self.ndof, self.T_MPC)))
        w_dq = 0.0001/float(self.T_MPC)
        w_dothers = 0.0001/float(self.T_MPC)
        for i in range(self.T_MPC):
            for j in range(self.T_MPC-1):
                dq_var_MPC[:, i] += (1./duration_MPC) * self.BC(self.n-1, j) * ((t[i] - t[0])/duration_MPC)**j * (1-((t[i] - t[0])/duration_MPC))**(self.n-1-j) * self.n * (Q[:, j+1] -  Q[:, j])
            if(i<(self.T_MPC -1)):
                name = 'control_point_deriv_' + str(i) + '_bound'  # add velocity constraint for each Q[:, i]
                builder_wholebodyMPC.add_bound_inequality_constraint(name, lhs=dlower, mid=(1./duration_MPC) * self.n * (Q[:, i+1] -  Q[:, i]), rhs=dupper)
            builder_wholebodyMPC.add_cost_term('minimize_velocity' + str(i), w_dq * optas.sumsqr(dq_var_MPC[:, i]))
        ##########################################################################################################################################################################
        self.h_com_larm = optas.casadi.SX(np.zeros((6, self.T_MPC)))
        self.h_com_rarm = optas.casadi.SX(np.zeros((6, self.T_MPC)))
        self.h_com_lleg = optas.casadi.SX(np.zeros((6, self.T_MPC)))
        self.h_com_rleg = optas.casadi.SX(np.zeros((6, self.T_MPC)))

        E = optas.casadi.SX(np.zeros(self.T_MPC))
        for i in range(self.T_MPC):
            for j in range(1, self.talos.link_num+1):
                v_temp = optas.casadi.SX(np.zeros(6)) # this the link velocity w.r.t. the base, expressed in body's local frame. 
                v_temp[0:3] = self.link_rot_wrt_base[j](q_var_MPC[:, i]).T @ (self.link_Jac_wrt_base[j](q_var_MPC[:, i]) @ dq_var_MPC[:, i])[3:6]
                v_temp[3:6] = self.link_rot_wrt_base[j](q_var_MPC[:, i]).T @ (self.link_Jac_wrt_base[j](q_var_MPC[:, i]) @ dq_var_MPC[:, i])[0:3]
                i_X_G_parallel_base = optas.casadi.SX(np.zeros((6,6)))
                i_X_G_parallel_base = self.i_X_G(self.link_pos_wrt_base[j](q_var_MPC[:, i]) - self.pos_com_wrt_base[:, i], self.link_rot_wrt_base[j](q_var_MPC[:, i]))
                if(j>=6 and j<=12): # left arm bodies
                    self.h_com_larm[:, i] += i_X_G_parallel_base.T @ self.talos.I[j] @ v_temp
                if(j>=13 and j<=19): # right arm bodies
                    self.h_com_rarm[:, i] += i_X_G_parallel_base.T @ self.talos.I[j] @ v_temp
                if(j>=20 and j<=25): # left leg bodies
                    self.h_com_lleg[:, i] += i_X_G_parallel_base.T @ self.talos.I[j] @ v_temp
                if(j>=26 and j<=31): # left leg bodies
                    self.h_com_rleg[:, i] += i_X_G_parallel_base.T @ self.talos.I[j] @ v_temp
                E[i] += v_temp.T @ self.talos.I[j] @ v_temp # calculate the energy
        #########################################################################################################################################################################
        # add position constraint at the beginning state
        builder_wholebodyMPC.add_equality_constraint('init_position', Q[0:8, 0], rhs=init_position_MPC[0:8])
        builder_wholebodyMPC.add_equality_constraint('init_position2', Q[10:self.ndof, 0], rhs=init_position_MPC[10:self.ndof])
        builder_wholebodyMPC.add_equality_constraint('head_miniscope', Q[8:10, :], rhs=np.zeros((2, self.T_MPC)))
        builder_wholebodyMPC.add_equality_constraint('init_velocity', dq_var_MPC[0:8, 0], rhs=init_velocity_MPC[0:8])
        builder_wholebodyMPC.add_equality_constraint('init_velocity2', dq_var_MPC[10:self.ndof, 0], rhs=init_velocity_MPC[10:self.ndof])
        #########################################################################################################################################################################
        # for i in range(self.T_MPC):
        #     # optimization cost: minimize energy
        #     builder_wholebodyMPC.add_cost_term('E' + str(i), E[i])
        #     builder_wholebodyMPC.add_leq_inequality_constraint('diagonal' + str(i), lhs=0, rhs=(self.h_com_larm[:, i] + self.h_com_rleg[:, i]).T @ (self.h_com_rarm[:, i] + self.h_com_lleg[:, i]))
        #     builder_wholebodyMPC.add_leq_inequality_constraint('twoleg' + str(i), lhs=0, rhs=self.h_com_rleg[:, i].T @ self.h_com_lleg[:, i])
        #########################################################################################################################################################################
        ddq_var_MPC = optas.casadi.SX(np.zeros((self.ndof, self.T_MPC)))
        w_ddq = 0.0005/float(self.T_MPC)
        w_ddothers = 0.0005/float(self.T_MPC)
        for i in range(self.T_MPC):
            for j in range(self.T_MPC-2):
                ddq_var_MPC[:, i] += (1./duration_MPC)**2 * self.BC(self.n-2, j) * ((t[i] - t[0])/duration_MPC)**j * (1-((t[i] - t[0])/duration_MPC))**(self.n-2-j) * self.n * (self.n-1)* (Q[:, j+2] -  2*Q[:, j+1] + Q[:, j])
            builder_wholebodyMPC.add_cost_term('minimize_acceleration' + str(i), w_ddq * optas.sumsqr(ddq_var_MPC[:, i]))
        #########################################################################################################################################################################
        # setup solver
        solver_options={'knitro.algorithm':1, 'knitro.linsolver':2,
                        'knitro.OutLev': 0,
                        'print_time': 0,
                        'knitro.FeasTol': 1e-6, 'knitro.OptTol': 1e-6, 'knitro.ftol':1e-6,
#                        'knitro.maxtime_real': 1.8e-2,
                        'knitro.bar_initpt':3, 'knitro.bar_murule':4, 'knitro.bar_penaltycons': 1,
                        'knitro.bar_penaltyrule':2, 'knitro.bar_switchrule':2, 'knitro.linesearch': 1}
        self.solver_wholebodyMPC = optas.CasADiSolver(optimization=builder_wholebodyMPC.build()).setup('knitro', solver_options)

        self.ti_MPC = 0 # time index of the MPC
        self.solution_MPC = None

        # self.I_G_conventional = [];
        # for i in range(self.T_MPC):
        #     self.I_G_conventional.append(optas.casadi.SX(np.zeros((6,6))))
        # for i in range(self.T_MPC):
        #     for j in range(1, self.talos.link_num+1):
        #         v_temp = optas.casadi.SX(np.zeros(6))
        #         v_temp[0:3] = (self.link_Jac[j](q_var_MPC[:, i]) @ dq_var_MPC[:, i])[3:6]
        #         v_temp[3:6] = (self.link_Jac[j](q_var_MPC[:, i]) @ dq_var_MPC[:, i])[0:3]
        #         i_X_G = optas.casadi.SX(np.zeros((6,6)))
        #         i_X_G = self.i_X_G(self.link_pos[j](q_var_MPC[:, i]) - self.pos_com[:, i], self.link_rot[j](q_var_MPC[:, i]))
        #         self.I_G_conventional[i] += i_X_G.T @ self.talos.I[j] @ i_X_G
        #         self.h_com[:, i] += i_X_G.T @ self.talos.I[j] @ i_X_G @ v_temp
        #         self.l_com[:, i] += self.h_com[3:6, i]
        #         self.k_com[:, i] += self.h_com[0:3, i]

        ### ---------------------------------------------------------
        # declare joint subscriber
        self._joint_sub = rospy.Subscriber("/joint_states", JointState, self.read_joint_states_cb)
        self._joint_sub_base = rospy.Subscriber("/talos/base_pose_ground_truth", Odometry, self.read_base_states_cb)
        self._joint_pub = rospy.Publisher("/talos/trajectory_controller/command", JointTrajectory, queue_size=10)
        self._joint_acc_pub = rospy.Publisher("/talos/joint_acc_pub", Float64MultiArray, queue_size=10)
        # set mux controller selection as wrong by default
        self._correct_mux_selection = False
        # declare mux service
        self._srv_mux_sel = rospy.ServiceProxy(rospy.get_namespace() + '/mux_joint_position/select', MuxSelect)
        # declare subscriber for selected controller
        self._sub_selected_controller = rospy.Subscriber("/mux_selected", String, self.read_mux_selection)
        # initialize action messages
        self._feedback = CmdTalosPoseFeedback()
        self._result = CmdTalosPoseResult()
        # declare action server
        self._action_server = actionlib.SimpleActionServer('cmd_pose', CmdTalosPoseAction, execute_cb=None, auto_start=False)
        # register the preempt callback
        self._action_server.register_goal_callback(self.goal_cb)
        self._action_server.register_preempt_callback(self.preempt_cb)
        # start action server
        self._action_server.start()

    def goal_cb(self):
        # activate publishing command
        self._srv_mux_sel(self._pub_cmd_topic_name)
        # accept the new goal request
        acceped_goal = self._action_server.accept_new_goal()
        # desired end-effector position
        pos_base_target = np.asarray([acceped_goal.poseBase.position.x, acceped_goal.poseBase.position.y, acceped_goal.poseBase.position.z])
        quat_base_target = np.asarray([acceped_goal.poseBase.orientation.x, acceped_goal.poseBase.orientation.y, acceped_goal.poseBase.orientation.z, acceped_goal.poseBase.orientation.w])
        self.stiffness_spring_com_target = acceped_goal.stiffness_spring_com
        self.rest_length_spring_com_target = acceped_goal.rest_length_spring_com
        self.duration = acceped_goal.duration
        self.duration_MPC = (np.pi * 2)/(math.sqrt(self.stiffness_spring_com_target/self.talos.m_G))
        # self.time_linspace = np.linspace(0., self.duration_MPC, self.T_MPC)
        # self.timebyT = np.asarray(self.time_linspace)/self.duration_MPC
        # self.time_linspace_periodic = np.linspace(0., self.duration_MPC*2, self.T_MPC*2)
        # self.timeby2T_periodic = np.asarray(self.time_linspace_periodic)/(self.duration_MPC*2)
        self.time = []
        # self.time_periodic = []
        # print goal request
        rospy.loginfo("%s: Request to send CoM to position (%.2f, %.2f, %.2f) with orientation (%.2f, %.2f, %.2f, %.2f), in %.1f seconds." % (self._name,
                pos_base_target[0], pos_base_target[1], pos_base_target[2], quat_base_target[0], quat_base_target[1], quat_base_target[2], quat_base_target[3], acceped_goal.duration))
        # read current robot joint positions
        self.q_curr = np.concatenate((self.q_curr_base, self.q_curr_joint), axis=None)
        self.dq_curr = np.concatenate((self.dq_curr_base, self.dq_curr_joint), axis=None)
        self.joint_names = self.joint_names_base + self.joint_names_position
        ### optas
        ### ---------------------------------------------------------
        # get two-arm end effector trajectory in the operational space
        q0 = self.q_curr.T
        self._steps = int(self.duration * self._freq)
        self._idx = 0
        # current right and left arm end effector position and quaternion
        start_base_pos = np.asarray(self.link_pos[1](self.q_curr))[:, 0]
        start_base_quat = np.asarray(self.link_quat[1](self.q_curr))[:, 0]
        # derivation of right and left arm end effector position and quaternion compared with the beginning ee position and quaternion
        Derivation_Base_Pos = pos_base_target - start_base_pos
        Derivation_Base_Quat = quat_base_target - start_base_quat
        # interpolate between current and target position polynomial obtained for zero speed (3rd order) and acceleratin (5th order) at the initial and final time
        self._base_Pos_trajectory = lambda t: start_base_pos + (10.*((t/self.duration)**3) - 15.*((t/self.duration)**4) + 6.*((t/self.duration)**5))*Derivation_Base_Pos # 5th order
        self._Dbase_Pos_trajectory = lambda t: (30.*((t/self.duration)**2) - 60.*((t/self.duration)**3) +30.*((t/self.duration)**4))*(Derivation_Base_Pos/self.duration)
        self._base_Quat_trajectory = lambda t: start_base_quat + (10.*((t/self.duration)**3) - 15.*((t/self.duration)**4) + 6.*((t/self.duration)**5))*Derivation_Base_Quat # 5th order
        self._Dbase_Quat_trajectory = lambda t: (30.*((t/self.duration)**2) - 60.*((t/self.duration)**3) +30.*((t/self.duration)**4))*(Derivation_Base_Quat/self.duration)

        self._com_Pos_z_trajectory = lambda t: -(self.talos.m_G * 9.81)/self.stiffness_spring_com_target * math.cos(math.sqrt(self.stiffness_spring_com_target/self.talos.m_G) * t) + self.rest_length_spring_com_target + (self.talos.m_G * 9.81)/self.stiffness_spring_com_target;
        
        self._t = np.linspace(0., self.duration, self._steps + 1)
        ### ---------------------------------------------------------
        # initialize the message
        self._msg = Float64MultiArray()
        self._msg.layout = MultiArrayLayout()
        self._msg.layout.data_offset = 0
        self._msg.layout.dim.append(MultiArrayDimension())
        self._msg.layout.dim[0].label = "columns"
        self._msg.layout.dim[0].size = self.ndof_joints + 2

        # initialize the message
        self._msg_acceleration = Float64MultiArray()
        self._msg_acceleration.layout = MultiArrayLayout()
        self._msg_acceleration.layout.data_offset = 0
        self._msg_acceleration.layout.dim.append(MultiArrayDimension())
        self._msg_acceleration.layout.dim[0].label = "columns"
        self._msg_acceleration.layout.dim[0].size = self.ndof + 2

        self.eva_trajectory = JointTrajectory()
        self.eva_trajectory.header.frame_id = ''
        self.eva_trajectory.joint_names = [
            'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint',
            'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint',
            'gripper_left_joint', 'gripper_right_joint', 'head_1_joint', 'head_2_joint',
            'leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint',
            'leg_right_1_joint', 'leg_right_2_joint', 'leg_right_3_joint', 'leg_right_4_joint', 'leg_right_5_joint', 'leg_right_6_joint',
            'torso_1_joint', 'torso_2_joint']
        self.eva_point = JointTrajectoryPoint()
        self.eva_point.time_from_start = rospy.Duration(0.1)
        self.eva_trajectory.points.append(self.eva_point)

        self.curr_MPC = np.zeros((self.ndof, self.T_MPC))
        for i in range(self.T_MPC):
            self.curr_MPC[:,i] = self.q_curr

        # create timer
        dur = rospy.Duration(1.0/self._freq)
        self._timer = rospy.Timer(dur, self.timer_cb)



    def timer_cb(self, event):
        # read current robot joint positions
        self.q_curr = np.concatenate((self.q_curr_base, self.q_curr_joint), axis=None)
        self.dq_curr = np.concatenate((self.dq_curr_base, self.dq_curr_joint), axis=None)

        # make sure that the action is active
        if(not self._action_server.is_active()):
            self._timer.shutdown()
            rospy.logwarn("%s: The action server is NOT active!")
            self._result.reached_goal = False
            self._action_server.set_aborted(self._result)
            return

        # main execution
        if(self._idx < self._steps):
            if(self._correct_mux_selection):
                # increment idx (in here counts starts with 1)
                self._idx += 1

                pos_base_goal = []; quat_base_goal = []; pos_com_goal_z = [];

                self.ti_MPC = 0

                for i in range(self.T_MPC):
#                    self.ti_MPC = self._t[self._idx-1]  + self.dt_MPC*i
                    if(self.ti_MPC <= self.duration):
                        self.ti_MPC = self._t[self._idx-1]  + self.dt_MPC*i
                    if(self.ti_MPC > self.duration):
                        self.ti_MPC = self.duration
                    try:
                        g_base_pos = self._base_Pos_trajectory(self.ti_MPC).flatten()
                        pos_base_goal.append(g_base_pos.tolist())
                        g_base_quat = self._base_Quat_trajectory(self.ti_MPC).flatten()
                        quat_base_goal.append(g_base_quat.tolist())
                        g_com_pos_z = self._com_Pos_z_trajectory(self.ti_MPC).flatten()
                        pos_com_goal_z.append(g_com_pos_z.tolist())
                    except ValueError:
                        pos_base_goal.append(g_base_pos.tolist()) # i.e. previous goal
                        quat_base_goal.append(g_base_quat.tolist()) # i.e. previous goal
                        pos_com_goal_z.append(g_com_pos_z.tolist()) # i.e. previous goal
                pos_base_goal = optas.np.array(pos_base_goal).T
                quat_base_goal = optas.np.array(quat_base_goal).T
                pos_com_goal_z = optas.np.array(pos_com_goal_z).T

            
                for i in range(self.T_MPC):
                    self.time.append(self._t[self._idx-1]+ self.dt_MPC*i)

                # for i in range(self.T_MPC*2-1):
                #     self.time_periodic.append(self._t[self._idx-1]+ self.dt_MPC*i)

                ### optas
                ### ---------------------------------------------------------
                ### solve the whole-body MPC
                # set initial seed
                if self.solution_MPC is None:
                    self.solver_wholebodyMPC.reset_initial_seed({f'Q': self.curr_MPC})            # set initial seed
                if self.solution_MPC is not None:
                    self.solver_wholebodyMPC.reset_initial_seed({f'Q': self.solution_MPC[f'Q'] })

                self.solver_wholebodyMPC.reset_parameters({'pos_base_goal': pos_base_goal, 'quat_base_goal': quat_base_goal, 'pos_com_goal_z': pos_com_goal_z, 
                                                           't': self.time, 'init_position_MPC': self.q_curr, 'init_velocity_MPC': self.dq_curr, 
                                                           'duration_MPC': self.duration_MPC } )

                # solve problem
                self.solution_MPC = self.solver_wholebodyMPC.opt.decision_variables.vec2dict(self.solver_wholebodyMPC._solve())
                Q = np.asarray(self.solution_MPC[f'Q'])

                self.time = []
                # self.time_periodic = []

                ### ---------------------------------------------------------
                # compute next configuration with lambda function
                t = (1./self._freq)/self.duration_MPC
                n = self.T_MPC -1
                q_next = np.zeros(self.ndof)
                for j in range(self.T_MPC):
                    q_next += self.BC(n, j) * t**j * (1-t)**(n-j) * Q[:, j]
                dq_next = np.zeros(self.ndof)
                for j in range(self.T_MPC-1):
                    dq_next += (1./self.duration_MPC) * self.BC(n-1, j) * t**j * (1-t)**(n-1-j) * n * (Q[:, j+1] -  Q[:, j])

                ddq_next = np.zeros(self.ndof)
                for j in range(self.T_MPC-2):
                    ddq_next += (1./self.duration_MPC)**2 * self.BC(n-2, j) * t**j * (1-t)**(n-2-j) * n * (n-1)* (Q[:, j+2] -  2*Q[:, j+1] + Q[:, j])

                # update message
                self.q_joints_output[0:14] = q_next[10:24];                 self.ddq_joints_output[0:14] = ddq_next[10:24];
                self.q_joints_output[14:16] = np.zeros(2);                  self.ddq_joints_output[14:16] = np.zeros(2);
                self.q_joints_output[16:18] = q_next[8:10];                 self.ddq_joints_output[16:18] = ddq_next[8:10];
                self.q_joints_output[18:30] = q_next[24:self.ndof];         self.ddq_joints_output[18:30] = ddq_next[24:self.ndof];
                self.q_joints_output[30:32] = q_next[6:8];                  self.ddq_joints_output[30:32] = ddq_next[6:8];

                self.eva_trajectory.header.stamp = rospy.Time(0)
                self.eva_trajectory.points[0].positions = self.q_joints_output.tolist()

                self._msg_acceleration.data = self.ddq_joints_output
                # publish message
                self._joint_pub.publish(self.eva_trajectory)
#                self._joint_pub.publish(self._msg)
                self._joint_acc_pub.publish(self._msg_acceleration)
                # compute progress
                self._feedback.progress = (self._idx*100)/self._steps
                # publish feedback
                self._action_server.publish_feedback(self._feedback)
            else:
                # shutdown this timer
                self._timer.shutdown()
                rospy.logwarn("%s: Request aborted. The controller selection changed!" % (self._name))
                self._result.reached_goal = False
                self._action_server.set_aborted(self._result)
                return
        else:
            # shutdown this timer
            self._timer.shutdown()
            # set the action state to succeeded
            rospy.loginfo("%s: Succeeded" % self._name)
            self._result.reached_goal = True
            self._action_server.set_succeeded(self._result)

            return

    def read_joint_states_cb(self, msg):
        self.q_curr_joint_fb = np.asarray(list(msg.position)[:self.ndof_joints+2])
        self.dq_curr_joint_fb = np.asarray(list(msg.velocity)[:self.ndof_joints+2]) # since in real gazebo, there are two gripper joints.

        self.q_curr_joint[4:18] = self.q_curr_joint_fb[0:14];                  self.dq_curr_joint[4:18] = self.dq_curr_joint_fb[0:14];
        self.q_curr_joint[2:4] = self.q_curr_joint_fb[16:18];                  self.dq_curr_joint[2:4] = self.dq_curr_joint_fb[16:18];
        self.q_curr_joint[18:self.ndof] = self.q_curr_joint_fb[18:30];         self.dq_curr_joint[18:self.ndof] = self.dq_curr_joint_fb[18:30];
        self.q_curr_joint[0:2] = self.q_curr_joint_fb[30:32];                  self.dq_curr_joint[0:2] = self.dq_curr_joint_fb[30:32];

#    def read_ft_sensor_right_data_cb(self, msg):
#        """ paranet to child: the force/torque from robot to ee"""
#        self.ft_right = -np.asarray([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

#    def read_ft_sensor_left_data_cb(self, msg):
#        """ paranet to child: the force/torque from robot to ee"""
#        self.ft_left = -np.asarray([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z, msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def read_base_states_cb(self, msg):
        base_euler_angle = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.q_curr_base = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, base_euler_angle[0], base_euler_angle[1], base_euler_angle[2]]
        self.donkey_R = optas.spatialmath.rotz(base_euler_angle[2]) @ optas.spatialmath.roty(base_euler_angle[1]) @ optas.spatialmath.rotx(base_euler_angle[0])
        self.donkey_position = np.asarray([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.donkey_velocity = np.asarray([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        self.donkey_angular_velocity = np.asarray([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        self.dq_curr_base = [float(msg.twist.twist.linear.x),  float(msg.twist.twist.linear.y),  float(msg.twist.twist.linear.z),
                             float(msg.twist.twist.angular.x), float(msg.twist.twist.angular.y), float(msg.twist.twist.angular.z)]

#    def read_base_states_cb(self, msg):
#        try:
#            (trans,rot) = self.tf_listener.lookupTransform('/vicon/world', 'vicon/talos/CHONK', rospy.Time(0))
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            print("error: cannot find vicon data!!!!")
#        self.base_euler_angle = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
#        self.q_curr_base = np.asarray([trans[0], trans[1], self.base_euler_angle[2]])
#        self.donkey_R = optas.spatialmath.rotz(self.base_euler_angle[2])

#        self.donkey_position = np.asarray([trans[0], trans[1], trans[2]])
#        self.donkey_velocity = self.donkey_R @ np.asarray([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
#        self.donkey_angular_velocity = self.donkey_R @ np.asarray([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z/6.05])
#        self.dq_curr_base = np.asarray([self.donkey_velocity[0], self.donkey_velocity[1], self.donkey_angular_velocity[2]])


    def read_mux_selection(self, msg):
        self._correct_mux_selection = (msg.data == self._pub_cmd_topic_name)

    def preempt_cb(self):
        rospy.loginfo("%s: Preempted.", self._name)
        # set the action state to preempted
        self._action_server.set_preempted()

    def BC(self, n, i):
        return np.math.factorial(n)/(np.math.factorial(i) * (np.math.factorial(n-i)))

    def skew(self, vec):
        A = np.asarray([ [0, -vec[2], vec[1]], [vec[2], 0, -vec[0]], [-vec[1], vec[0], 0]])
        return A

    def G_X_i(self, G_p_i, G_R_i):
        X = optas.casadi.SX(np.zeros((6, 6)))

        X[0:3, 0:3] = G_R_i
        X[0:3, 3:6] = np.zeros((3,3))
        X[3:6, 0:3] = optas.spatialmath.skew(G_p_i) @ G_R_i
        X[3:6, 3:6] = G_R_i
        return X

    def i_X_G(self, G_p_i, G_R_i):
        X = optas.casadi.SX(np.zeros((6, 6)))

        X[0:3, 0:3] = G_R_i.T
        X[0:3, 3:6] = np.zeros((3,3))
        X[3:6, 0:3] = -G_R_i.T @ optas.spatialmath.skew(G_p_i)
        X[3:6, 3:6] = G_R_i.T
        return X
    
    def i_X_b(self, b_p_i, b_R_i):
        X = optas.casadi.SX(np.zeros((6, 6)))

        X[0:3, 0:3] = b_R_i.T
        X[0:3, 3:6] = np.zeros((3,3))
        X[3:6, 0:3] = -b_R_i.T @ optas.spatialmath.skew(b_p_i)
        X[3:6, 3:6] = b_R_i.T
        return X
    
    def qaQb(self, a, b):
        Quaternion_result = optas.casadi.SX(np.zeros(4))
        Quaternion_result[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1]
        Quaternion_result[1] = a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2]
        Quaternion_result[2] = a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0]
        Quaternion_result[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]
        return Quaternion_result

if __name__=="__main__":
    # Initialize node
    rospy.init_node("cmd_pose_server_MPC", anonymous=True)
    # Initialize node class
    cmd_pose_server = CmdPoseActionServer(rospy.get_name())
    # executing node
    rospy.spin()
