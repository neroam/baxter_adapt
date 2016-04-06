#!/usr/bin/env python
NAME = 'imitation_server'

from baxter_adapt.srv import *
import rospy

def test(req):
    print req.y_start
    print req.y_end
    resp = ImitationResponse()
    resp.response = True
    resp.filename = "Test"
    return resp

def imitation_server():
    rospy.init_node(NAME)
    s = rospy.Service('baxter_adapt/imitation_server', Imitation, test)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    imitation_server()
