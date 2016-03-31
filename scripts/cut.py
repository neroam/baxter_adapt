#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Baxter RSDK Joint Torque Example: joint springs
"""

import argparse

import rospy

from dynamic_reconfigure.server import (
    Server,
)
from std_msgs.msg import (
    Empty,
)

import baxter_interface

from baxter_examples.cfg import (
    JointSpringsExampleConfig,
)
from baxter_interface import CHECK_VERSION

from baxter_pykdl import baxter_kinematics

from execution import *

import cma

import numpy as np

import sys


class JointSprings(object):
    """
    Virtual Joint Springs class for torque example.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server

    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, limb, reconfig_server):
        self._dyn = reconfig_server

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
        self._z_cut_through = 0.020 # 0.0328

        # parameters for calculating the accelaration
        self._last_time = 0
        self._last_v = 0

        # score and time
        self._score = 0
        self._time_consumption = 0

        # result
        self.result = []

        # create our limb instance
        self._side = limb
        self._limb = baxter_interface.Limb(limb)
        self._kin = baxter_kinematics(limb)
        self._jacob = self._kin.jacobian_pseudo_inverse()

        self.joint_states = {

            'observe':{
                'left_e0': -0.662,
                'left_e1': 1.65,
                'left_s0': 0.0423,
                'left_s1': -0.878,
                'left_w0': 0.53,
                'left_w1': 1.06,
                'left_w2': -0.0706,
            }
        }


        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)
        self.pub = rospy.Publisher('position_error', Point, queue_size=10)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def initialize(self):
        self._score = 0
        self._time_consumption = 0
        self._last_time = 0
        set_joints(target_angles_left = self.joint_states['observe'])
        ik_move('left', self.pub, self._jacob, control_mode = 'position', target_dz = -0.06, speed = 0.2, timeout = 20)
        rospy.sleep(5)

    def recover(self):
        ik_move('left', self.pub, self._jacob, control_mode = 'position', target_dz = 0.2, speed = 0.5, timeout = 20)

    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] +
                                                    '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] +
                                                    '_damping_coefficient']

    def _update_forces(self, initial_pose, target_x, target_y, target_z):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # get latest spring constants
        self._update_parameters()

        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        current_pose = get_current_pose(self._limb, initial_pose)
        current_joints = self._limb.joint_angles()
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_z = current_pose.pose.position.z
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z

        #dz = min(-0.05, dz)
        target_pose = update_current_pose(current_pose, dx, dy, dz)
        # target_joints = ik_joint(target_pose, self._side)
        target_joints = ik_pykdl_joint(self._kin, target_pose, side = self._side)
        cur_vel = self._limb.joint_velocities()
        p = Point()
        p.x = dx * 1e6
        p.y = dy * 1e6
        p.z = dz * 1e6
        self.pub.publish(p)
        # calculate current forces
        if len(current_joints) is not 0 and len(target_joints) is not 0:
            for joint in self._start_angles.keys():
                # spring portion
                cmd[joint] = self._springs[joint] * (target_joints[joint] -
                                                       current_joints[joint])
                # damping portion
                # cmd[joint] -= self._damping[joint] * cur_vel[joint]
            # command new joint torques
            cmd['left_w2'] = 0.0
            self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.-
        """
        self._limb.move_to_neutral()

    def attach_springs(self, theta, dx = 0.0, dy = 0.0, dz = 0.03): # dz = 0.03
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        initial_pose = get_current_pose(self._limb)
        target_x = initial_pose.pose.position.x + dx
        target_y = initial_pose.pose.position.y + dy
        target_z = initial_pose.pose.position.z + dz

        force_max = -30

        time = []
        force = []
        # loop at specified rate commanding new joint torques
        currnet_z = get_current_pose(self._limb).pose.position.z
        while not rospy.is_shutdown() and currnet_z > self._z_cut_through and currnet_z < 0.1:
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            self._update_forces(initial_pose, target_x, target_y, target_z)
            delta_x = 0.0
            delta_z = self.generate_delta_z(theta)
            target_z += delta_z
            target_x += delta_x
            currnet_z = get_current_pose(self._limb).pose.position.z
            time.append(rospy.get_time())
            force.append(self._limb.endpoint_effort()['force'].z)
            control_rate.sleep()
        if currnet_z >= 0.1: self._score = -100
        force_profile = np.asarray([time, force])
        self.output(force_profile)

    def generate_delta_z(self, theta):
        x = self._limb.endpoint_pose()['position'].z - self._z_cut_through
        v = self._limb.endpoint_velocity()['linear'].z
        t = rospy.get_time()
        dt =  0 if self._last_time == 0 else t - self._last_time
        a = 0.0 if self._last_time == 0 else (v - self._last_v) / dt

        # update last_time and last_v
        self._last_v = v
        self._last_time = t

        # update score and time
        self._score += -abs(self._limb.endpoint_effort()['force'].z) * abs(v) * dt * 100
        self._time_consumption += dt
        return theta[0] * x + theta[1] * v + theta[2] * a + theta[3]

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def output(self, force_profile):
        print "Fianl socre: ", self._score
        print "Total time: ", self._time_consumption
        print len(force_profile[1])
        np.save('/home/aecins/force_profile', force_profile)

    def accept(self, theta):
        x = np.asarray([theta, self._score, self._time_consumption])
        np.save('/home/aecins/output', x)



def main():


    rospy.init_node("robot_cut")
    # theta = []
    # for s in [sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]]:
    #     theta.append(float(s))
    theta = [-0.0003297, 9.12e-6, -3.9558e-5, -0.000209 ]


    dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)
    cut(theta, dynamic_cfg_srv)


def cut(theta, dsv):

    js = JointSprings('left', dsv)
    js.initialize()
    print "Executed theta: ", theta
    js.attach_springs(theta)
    js.recover()
    js.accept(theta)






def test():
    rospy.init_node('cut_test', anonymous=True)

    joint_states = {

        'observe':{
            'left_e0': 0.06442719301757813,
            'left_e1': 1.3721458131958009,
            'left_s0':  0.44715539915771485,
            'left_s1': -0.8076408838989259,
            'left_w0': -0.061742726641845706,
            'left_w1': 1.0239321747436525,
            'left_w2': 1.4868108769592285,
        }
    }


    set_joints(target_angles_left = joint_states['observe'])
    rospy.sleep(1)
    pub = rospy.Publisher('position_error', Point, queue_size=10)
    jacob = baxter_kinematics('left').jacobian_pseudo_inverse()
    for i in range(3):
        ik_move('left', pub, jacob, control_mode = 'position', target_dz = -0.2, speed = 1.0, timeout = 20)
        ik_move('left', pub, jacob, control_mode = 'position', target_dz = 0.2, speed = 1.0, timeout = 20)


if __name__ == "__main__":
    #main()
    test()
