#!/usr/bin/env python

import sys
import os

import rospy

from baxter_adapt.srv import *
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

def adaptation_client(x, y, obstacles):

    rospy.wait_for_service('baxter_adapt/adaptation_server', 20.0)

    try:
        adaptation = rospy.ServiceProxy('baxter_adapt/adaptation_server', Adaptation)

        # simplified style
        resp1 = adaptation(x, y, obstacles)
        print resp1.response
        print resp1.filename

        while os.path.isfile(resp1.filename) is False:
            print "Wait for trajectory generation\n"
            rospy.sleep(2.0)
        print "Trajectory generated\n"


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    import random
    x = Point(0.2, 0.3, 0.5)
    y = Point(0.1, 0.4, 0.2)
    obstacles = [Point(0.25, 0.33, 0.4), Point(0.12,0.24,0.22)]
    adaptation_client(x, y, obstacles)
