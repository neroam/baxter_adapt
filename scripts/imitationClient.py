#!/usr/bin/env python

import sys
import os

import rospy

from baxter_adapt.srv import *

def imitation_client(x, y):

    rospy.wait_for_service('imitation_server')

    try:
        imitation = rospy.ServiceProxy('imitation_server', Imitation)
        print x
        print y

        # simplified style
        resp1 = imitation(x, y)
        print resp1.response
        print resp1.filename

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    import random
    x = [0.2,0.3,0.5]
    y = [0.1,0.4,0.2]
    imitation_client(x, y)
