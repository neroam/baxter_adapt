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

def imitation_client(x, y):

    rospy.wait_for_service('baxter_adapt/imitation_server', 20.0)

    try:
        imitation = rospy.ServiceProxy('baxter_adapt/imitation_server', Imitation)

        # simplified style
        resp1 = imitation(x, y)
        print resp1.response
        print resp1.filename

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    import random
    x = Point(0.2, 0.3, 0.5)
    y = Point(0.1, 0.4, 0.2)
    imitation_client(x, y)
