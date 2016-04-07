#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None

def clean_line(line, names):
    """
    Cleans a single line of recorded joint positions

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    #convert the line of strings to a float or None
    line = [try_float(x) for x in line.rstrip().split(',')]
    #zip the values with the joint names
    combined = zip(names[1:], line[1:])
    #take out any tuples that have a none value
    cleaned = [x for x in combined if x[1] is not None]
    #convert it to a dictionary with only valid commands
    command = dict(cleaned)
    left_command = dict((key, command[key]) for key in command.keys()
                        if key[:5] == 'left_')
    right_command = dict((key, command[key]) for key in command.keys()
                         if key[:6] == 'right_')
    return (command, left_command, right_command, line)

class Executor(object):
    def __init__(self, limb, verbose=True):
        self._limb_name = limb # string
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._gripper.calibrate()
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def set_to_joint(self, angles):
        self._limb.set_joint_positions(angles)

    def move_to_joint(self, angles):
        self._guarded_move_to_joint_position(angles)

    def get_current_pose(self):
        pose = self._limb.endpoint_pose()
        while len(pose) == 0:
            print "Error"
            rospy.sleep(1)
            pose = arm.endpoint_pose()
        ep_position = pose['position']
        ep_orientation = pose['orientation']

        current_pose = Pose(
                        position=Point(
                            x=ep_position.x,
                            y=ep_position.y,
                            z=ep_position.z,
                        ),
                        orientation=Quaternion(
                            x=ep_orientation.x,
                            y=ep_orientation.y,
                            z=ep_orientation.z,
                            w=ep_orientation.w,
                        ),
                      )

        return current_pose

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def move_to_pose(self, pose):
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def move_as_trajectory(self, filename):
        joints_left = self._limb.joint_names()
        rate = rospy.Rate(1000)

        print("Calculating Trajectory: %s" % (filename,))
        lines = self.ik_trajectory(filename, filename + '_joints')
        keys = lines[0].rstrip().split(',')

        cmd, lcmd, rcmd, values = clean_line(lines[1], keys)
        print("Performing Trajectory")
        self.move_to_joint(lcmd)

        start_time = rospy.get_time()
        step = 1
        for i in xrange(1, len(lines)-1, step):
            values = lines[i]
            sys.stdout.write("\r Trajectory %d of %d" %
                             (i, len(lines) - 1))
            sys.stdout.flush()

            cmd, lcmd, rcmd, values = clean_line(values, keys)
            #command this set of commands until the next frame
            while (rospy.get_time() - start_time) < values[0] :
                if rospy.is_shutdown():
                    print("\n Aborting - ROS shutdown")
                    return False
                if len(lcmd):
                    self.move_to_joint(lcmd)
                rate.sleep()

    def ik_trajectory(self, file_in, file_out):

        joints_left = self._limb.joint_names()

        with open(file_in, 'r') as f:
            lines = f.readlines()
        keys = lines[0].rstrip().split(',')

        f =  open(file_out, 'w')
        lines_out = []
        lines_out.append('time,' + ','.join([j for j in joints_left]) + '\n')
        f.write(lines_out[0])

        _cmd, lcmd_start, rcmd_start, _raw = clean_line(lines[1], keys)

        target = copy.deepcopy(self.get_current_pose())
        target.position.x = lcmd_start['left_pos_x']
        target.position.y = lcmd_start['left_pos_y']
        target.position.z = lcmd_start['left_pos_z']

        """
        starting_joint_angles = {joint:lcmd_start[joint] for joint in joints_left}
        self.move_to_joint(starting_joint_angles)
        target = copy.deepcopy(self.get_current_pose())
        """
        step = 1
        for i in xrange(1, len(lines)-1, step):
            values = lines[i]
            sys.stdout.write("\r Trajectory %d of %d\n" %
                             (i, len(lines) - 1))
            sys.stdout.flush()

            cmd, lcmd, rcmd, values = clean_line(values, keys)
            #command this set of commands until the next frame
            if len(lcmd):
                target.position.x = lcmd['left_pos_x']
                target.position.y = lcmd['left_pos_y']
                target.position.z = lcmd['left_pos_z']
                joint_angles = self.ik_request(target)
                if joint_angles:
                    angles_left = [joint_angles[joint] for joint in self._limb.joint_names()]
                    print angles_left

                    new_line = '%f,' % values[0]
                    new_line += ','.join([str(x) for x in angles_left]) + '\n'
                    lines_out.append(new_line)
                    f.write(new_line)
        return lines_out






