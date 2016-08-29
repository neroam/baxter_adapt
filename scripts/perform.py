#!/usr/bin/env python

"""
Perform a trajectory according to file
"""

import argparse
import sys

import rospy
import copy

import baxter_interface
from baxter_interface import CHECK_VERSION

from executor import Executor

def map_file(filename):
    executor = Executor("right")
    traj = open(filename, 'r').readlines()
    #traj = executor.ik_trajectory(filename, filename + '_joints', executor.get_current_pose())
    executor.move_as_trajectory(traj)

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

    map_file(args.file)


if __name__ == '__main__':
    main()
