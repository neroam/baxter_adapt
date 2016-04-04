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
Baxter RSDK Joint Position Example: file playback
"""
import argparse
import sys

import rospy
import copy

import baxter_interface
from baxter_interface import CHECK_VERSION

from executor import Executor

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

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


def map_file(filename, loops=1):
    limb = "left"
    rate = rospy.Rate(1000)
    executor = Executor(limb)
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

    start_pose = Pose(
        position=Point(x=0.7, y=0.15, z=-0.129),
        orientation=overhead_orientation)

    executor.move_to_pose(start_pose)
    rospy.sleep(1)

    print("Performing Trajectory: %s" % (filename,))
    with open(filename, 'r') as f:
        lines = f.readlines()
    keys = lines[0].rstrip().split(',')

    l = 0
    # If specified, repeat the file playback 'loops' number of times
    while loops < 1 or l < loops:
        i = 0
        l += 1
        print("Moving to start position...")

        _cmd, lcmd_start, rcmd_start, _raw = clean_line(lines[1], keys)
        target = copy.deepcopy(start_pose)
        target.position.x = lcmd_start['left_pos_x']
        target.position.y = lcmd_start['left_pos_y']
        target.position.z = lcmd_start['left_pos_z']
        executor.move_to_pose(target)
        rospy.sleep(1)

        starting_joint_angles = {'left_w0': lcmd_start['left_w0'],
                             'left_w1': lcmd_start['left_w1'],
                             'left_w2': lcmd_start['left_w2'],
                             'left_e0': lcmd_start['left_e0'],
                             'left_e1': lcmd_start['left_e1'],
                             'left_s0': lcmd_start['left_s0'],
                             'left_s1': lcmd_start['left_s1']}

        executor.move_to_start(starting_joint_angles)
        target = copy.deepcopy(executor.get_current_pose())

        step = 20
        for i in xrange(1, len(lines)-1, step):
            start_time = rospy.get_time()
            values = lines[i]
            loopstr = str(loops) if loops > 0 else "forever"
            sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                             (i, len(lines) - 1, l, loopstr))
            sys.stdout.flush()

            cmd, lcmd, rcmd, values = clean_line(values, keys)
            #command this set of commands until the next frame
            while (rospy.get_time() - start_time) < 0.01 :
                if rospy.is_shutdown():
                    print("\n Aborting - ROS shutdown")
                    return False
                if len(lcmd):
                    target.position.x = lcmd['left_pos_x']
                    target.position.y = lcmd['left_pos_y']
                    target.position.z = lcmd['left_pos_z']
                    executor.move_to_pose(target)
                rate.sleep()
        print
    return True


def main():
    """RSDK Joint Position Example: File Playback

    Uses Joint Position Control mode to play back a series of
    recorded joint and gripper positions.

    """
    epilog = """
Related examples:
  joint_recorder.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='path to input file'
    )
    parser.add_argument(
        '-l', '--loops', type=int, default=1,
        help='number of times to loop the input file. 0=infinite.'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("baxter_perform_trajectory")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
    rospy.on_shutdown(clean_shutdown)

    map_file(args.file, args.loops)


if __name__ == '__main__':
    main()
