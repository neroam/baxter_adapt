#!/usr/bin/env python

"""
Baxter RSDK Inverse Kinematics adapt Demo
"""
import argparse
import struct
import sys
import copy
import os

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from executor import Executor
from baxter_adapt.srv import *

class Task(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._executor = Executor(limb, verbose)
        #trajsvc = "baxter_adapt/adaptation_server"
        trajsvc = "baxter_adapt/imitation_server"
        rospy.wait_for_service(trajsvc, 5.0)
        #self._trajsvc = rospy.ServiceProxy(trajsvc, Adaptation)
        self._trajsvc = rospy.ServiceProxy(trajsvc, Imitation)

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._executor.move_to_joint(start_angles)
        self._executor.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        self._executor.move_to_pose(approach)

    def _retract(self):
        # retrieve current pose from endpoint
        retract = copy.deepcopy(self._executor.get_current_pose())
        retract.position.z = retract.position.z + self._hover_distance
        self._executor.move_to_pose(retract)

    def transfer(self, pose, obstacles):
        # request Matlab to compute trajectory
        pose_start = self._executor.get_current_pose()
        y_start = pose_start.position
        y_end = pose.position
        resp = self._trajsvc(y_start, y_end, obstacles)
        print resp

        while os.path.isfile(resp.filename) is False:
            print "Wait for trajectory generation"
            rospy.sleep(2.0)

        self._executor.move_as_trajectory(resp.filename)

        #if resp.response is True:
            # perform the trajectory via Inverse Kinematics
            #self._executor.move_as_trajectory(resp.filename)

    def transfer_imitation(self, pose):
        pose_start = self._executor.get_current_pose()
        y_start = pose_start.position
        y_end = pose.position
        resp = self._trajsvc(y_start, y_end)
        print resp

        if resp.response is True:
            self._executor.move_as_trajectory(resp.filename)

    def pick(self, pose):
        print self._executor.get_current_pose()
        # open the gripper
        self._executor.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._executor.move_to_pose(pose)
        # close gripper
        self._executor.gripper_close()
        # retract to clear object
        #self._retract()

    def place(self, pose):
        # servo above pose
        #self._approach(pose)
        # servo to pose
        self._executor.move_to_pose(pose)
        # open the gripper
        self._executor.gripper_open()
        # retract to clear object
        self._retract()

def main():

    rospy.init_node("baxter_adapt_demo")

    #rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    tk = Task(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    side_orientation = Quaternion(
                             x=0.702323969299,
                             y=0.0288324448253,
                             z=0.709840848369,
                             w=0.0451187572977)
    block_poses = list()
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.45, z=-0.029),
        orientation=side_orientation))
    block_poses.append(Pose(
        position=Point(x=0.75, y=0.25, z=-0.029),
        orientation=side_orientation))

    obstacles = [Point(0.8, 0.1, 0)]

    #tk.move_to_start(starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
        print("\nPicking...")
        tk.pick(block_poses[idx])
        idx = (idx+1) % len(block_poses)
        print("\nTransferring...")
        #tk.transfer(block_poses[idx], obstacles)
        tk.transfer_imitation(block_poses[idx])
        print("\nPlacing...")
        tk.place(block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())
