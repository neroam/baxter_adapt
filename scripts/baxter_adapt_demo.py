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
        #trajsvc = "baxter_adapt/imitation_server"
        learningsvc = "baxter_adapt/learning_server"
        rospy.wait_for_service(trajsvc, 5.0)
        rospy.wait_for_service(learningsvc, 5.0)
        self._trajsvc = rospy.ServiceProxy(trajsvc, Adaptation)
        self._learningsvc = rospy.ServiceProxy(learningsvc, Learning)
        #self._trajsvc = rospy.ServiceProxy(trajsvc, Imitation)

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

    def transfer(self, pose_start, pose_end, obstacles):
        # request Matlab to compute trajectory
        y_start = pose_start.position
        y_end = pose_end.position

        print "Adapting Trajectory Learned from Demonstrations"
        resp = self._trajsvc(y_start, y_end, obstacles)
        print resp

        while os.path.isfile(resp.filename) is False:
            print "Wait for trajectory generation"
            rospy.sleep(2.0)

        print("Calculating Trajectory: %s" % (resp.filename,))
        traj = self._executor.ik_trajectory(resp.filename, resp.filename + '_joints', pose_start)

        self.pick(pose_start)
        rospy.sleep(1.0)
        self._executor.move_as_trajectory(traj)
        rospy.sleep(1.0)
        self.place(pose_end)

        #if resp.response is True:
            # perform the trajectory via Inverse Kinematics
            #self._executor.move_as_trajectory(resp.filename)

    def transfer_imitation(self, pose):
        pose_start = self._executor.get_current_pose()
        y_start = pose_start.position
        y_end = pose.position

        print y_start
        print y_end

        resp = self._trajsvc(y_start, y_end)
        print resp

        if resp.response is True:
            self._executor.move_as_trajectory(resp.filename)

    def pick(self, pose):
        # open the gripper
        self._executor.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        print self._executor.get_current_joints
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

    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0 # meters
    # Starting Joint angles for left arm
#     starting_joint_angles = {'left_w0': 0.6699952259595108,
#                              'left_w1': 1.030009435085784,
#                              'left_w2': -0.4999997247485215,
#                              'left_e0': -1.189968899785275,
#                              'left_e1': 1.9400238130755056,
#                              'left_s0': 0.58000397926829805,
#                              'left_s1': -0.9999781166910306}
#

    starting_joint_angles = {'left_w0': 0.3699952259595108,
                             'left_w1': -1.030009435085784,
                             'left_w2': -2.999997247485215,
                             'left_e0': -0.089968899785275,
                             'left_e1': 1.8400238130755056,
                             'left_s0': -0.18000397926829805,
                             'left_s1': -0.4999781166910306}


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

    print "Ready to go!"
    while not rospy.is_shutdown():

        # Update the task contexts via perception
        start_pose = Pose(
            position=Point(x=0.83, y=0.53, z=-0.10),
            orientation=side_orientation)

        end_pose = Pose(
            position=Point(x=0.80, y=-0.08, z=-0.11),
            orientation=side_orientation)

        obstacles = [Point(0.83, 0.36, 0)]

        # Reset the robot pose
        tk.move_to_start(starting_joint_angles)

        # Transferring the object via adaptation movement
        print("\nTransferring...")
        tk.transfer(start_pose, end_pose, obstacles)
        #tk.transfer_imitation(end_pose)

        needs_improve = raw_input('Would you like to improve movement? (y/n)')
        if needs_improve == 'y':
            tk.move_to_start(starting_joint_angles)
            tk.pick(start_pose)
            tk.learning()

    return 0

if __name__ == '__main__':
    sys.exit(main())
