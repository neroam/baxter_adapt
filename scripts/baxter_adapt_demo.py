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
        trajsvc = "baxter_adapt/adaptation_server"
        learningsvc = "baxter_adapt/learning_server"
        rospy.wait_for_service(trajsvc, 5.0)
        rospy.wait_for_service(learningsvc, 5.0)
        self._trajsvc = rospy.ServiceProxy(trajsvc, Adaptation)
        self._learningsvc = rospy.ServiceProxy(learningsvc, Learning)

    def move_to_start(self, start_angles=None):
        print self._executor.get_current_joints()

        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._executor.move_to_joint(start_angles)
        self._executor.gripper_open()
        rospy.sleep(1.0)

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

    def transfer(self, start_pose, end_pose, obstacles):

        print("Picking up the object")
        self.pick(start_pose)

        joint_names = self._executor.joint_names()
        joints = self._executor.ik_request(start_pose)
        start_joints = [joints[i] for i in joint_names]
        joints = self._executor.ik_request(end_pose)
        end_joints = [joints[i] for i in joint_names]

        print start_joints
        print end_joints

        print "Adapting Trajectory Learned from Demonstrations"
        resp = self._trajsvc(start_joints, end_joints, obstacles, True)
#         resp = AdaptationResponse()
#         resp.filename = '/home/aecins/ros_ws/src/baxter_adapt/MPC/generated/JointsTrajectory'
#         resp.response = True
        print resp

        if resp.response is True:
            while os.path.isfile(resp.filename) is False:
                print "Wait for trajectory generation"
                rospy.sleep(2.0)
            print("Perform trajectory")
            traj = open(resp.filename, 'r').readlines()
            self._executor.move_as_trajectory(traj)

    def transfer_imitation(self, start_pose, end_pose):
        print("Picking up the object")
        self.pick(start_pose)

        print("Computing imitation trajectory")
        joint_names = self._executor.joint_names()
        joints = self._executor.ik_request(start_pose)
        start_joints = [joints[i] for i in joint_names]
        joints = self._executor.ik_request(end_pose)
        end_joints = [joints[i] for i in joint_names]

        print start_joints
        print end_joints

        obstacles = []
        resp = self._trajsvc(start_joints, end_joints, obstacles, False)
        print resp

        if resp.response is True:
            print("Perform trajectory")
            traj = open(resp.filename, 'r').readlines()
            self._executor.move_as_trajectory(traj)

    def get_current_joints(self):
        return self._executor.get_current_joints()

    def get_current_pose(self):
        return self._executor.get_current_pose()

    def pick(self, pose):
        # open the gripper
        self._executor.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._executor.move_to_pose(pose)
        # close gripper
        self._executor.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        #self._approach(pose)
        # servo to pose
        self._executor.move_to_pose(pose)
        # open the gripper
        self._executor.gripper_open()
        # retract to clear object
        self._retract()

    def learning(self):
        print('Learning weights from feedback')
        feedback = self.get_feedback()
        resp = self._learningsvc(feedback)
        print('Updated weights for movement adaptation')

    def get_feedback(self):
        filename = os.getcwd() + '/trajectory_improved'
        print('Please help me improve the task.')
        self._executor.record(filename)
        print('Done')
        return filename

def main():

    rospy.init_node("baxter_adapt_demo")
    print "Initializing..."

    #rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0 # meters
    # Starting Joint angles for left arm

    # bowl bottle experiment
    starting_joint_angles = {'left_w0': -0.4649,
                             'left_w1': -0.3395,
                             'left_w2': 0.3267,
                             'left_e0': 1.2276,
                             'left_e1': -0.4221,
                             'left_s0': 0.7814,
                             'left_s1': 0.7059}

#     starting_joint_angles = {'left_w0': -0.23009711792,
#                              'left_w1': 0.749733109222,
#                              'left_w2': -1.41164581844,
#                              'left_e0': 0.192131093463,
#                              'left_e1': 1.27051958611,
#                              'left_s0': -0.37275733103,
#                              'left_s1': -0.415325297845}
#
#     starting_joint_angles = {'left_w0': 0.6699952259595108,
#                              'left_w1': 1.030009435085784,
#                              'left_w2': -0.4999997247485215,
#                              'left_e0': -1.189968899785275,
#                              'left_e1': 1.9400238130755056,
#                              'left_s0': 0.58000397926829805,
#                              'left_s1': -0.9999781166910306}
#
#     starting_joint_angles = {'left_w0': 0.3699952259595108,
#                              'left_w1': -1.030009435085784,
#                              'left_w2': -2.999997247485215,
#                              'left_e0': -0.089968899785275,
#                              'left_e1': 1.8400238130755056,
#                              'left_s0': -0.18000397926829805,
#                              'left_s1': -0.4999781166910306}
#
    tk = Task(limb, hover_distance)

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_start = Quaternion(
                             x=0.859512145991,
                             y=-0.510973150634,
                             z=-0.0120503637075,
                             w=-0.000314577239305)

    overhead_end = Quaternion(
                             x=0.876030936165,
                             y=-0.481110282735,
                             z=0.0317752280596,
                             w=-0.0096451858595)

    overhead = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)


    side = Quaternion(
                             x=0.702323969299,
                             y=0.0288324448253,
                             z=0.709840848369,
                             w=0.0451187572977)

    print "Ready to go!"

    start_pose = tk.get_current_pose()
    end_pose = copy.deepcopy(start_pose)
#bowlbottle before learning
#    end_pose.position.x += 0.1
 #   end_pose.position.y -= 0.4
#bowlbottle after learning
    end_pose.position.x += 0.15
    end_pose.position.y -= 0.45

    # bowl bottle experiment
    obstacles = [Point(0.72, 0.46, -0.07)]
    #after learning
    #obstacles = [Point(0.66, 0.30, -0.07)]

    while not rospy.is_shutdown():

        # Reset the robot pose
        tk.pick(start_pose)

        # Update the task contexts via perception
#         start_pose = Pose(
#             position=Point(x=0.64, y=0.59, z=-0.08),
#             orientation=overhead_end)
#
#         end_pose = Pose(
#             position=Point(x=0.63, y=0.02, z=-0.08),
#             orientation=overhead_end)
#

        # Transferring the object via adaptation movement
        print("\nTransferring...")
        tk.transfer(start_pose, end_pose, obstacles)
        #tk.transfer_imitation(start_pose, end_pose)

        needs_improve = raw_input('Would you like to improve movement? (y/n)')
        if needs_improve == 'y':
            tk.move_to_start(starting_joint_angles)
            tk.pick(start_pose)
            tk.learning()

    return 0

if __name__ == '__main__':
    sys.exit(main())
