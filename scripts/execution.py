import rospy
from baxter_interface import Limb
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import numpy as np
import math
import copy

def set_joints( target_angles_right = None, target_angles_left = None,timeout= 40000):

    right = Limb("right")
    left = Limb("left")
    
    if target_angles_right == None:
        reach_right = True
    else:
        reach_right = False
    

    if target_angles_left == None:
        reach_left = True
    else:
        reach_left = False
    
    time = 0

    while not reach_right or not reach_left:

            if target_angles_right: right.set_joint_positions(target_angles_right)
            if target_angles_left: left.set_joint_positions(target_angles_left)
            current_angles_right = right.joint_angles()
            current_angles_left = left.joint_angles()

            
            if reach_right == False:
                for k, v in current_angles_right.iteritems():
                    if abs(target_angles_right[k] - v) > 0.01:
                        reach_right = False
                        break
                    reach_right = True

            if reach_left == False:
                for k, v in current_angles_left.iteritems():
                    if abs(target_angles_left[k] - v) > 0.01:
                        reach_left = False
                        break
                    reach_left = True

            time+=1
            if time > timeout:
                print "Time out"
                break

def ik(pose, side):
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    if side == "left": ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 0

    if (resp.isValid[0]):

        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        if side == "right": arm = Limb("right")
        else: arm = Limb("left")
        arm.set_joint_positions(limb_joints)
        return 1

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 0

def ik_joint(pose, side):
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    if side == "left": ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return {}

    if (resp.isValid[0]):

        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return {}

def ik_pykdl(arm, kin, pose, side = "right"):
    position = pose.pose.position
    orientation = pose.pose.orientation
    pos = [position.x,position.y,position.z]
    rot = [orientation.x,orientation.y,orientation.z,orientation.w]
    joint_angles = kin.inverse_kinematics(pos,rot)
    if joint_angles:
        if side == "right":
            cmd = {
                'right_s0': joint_angles[0],
                'right_s1': joint_angles[1],
                'right_e0': joint_angles[2],
                'right_e1': joint_angles[3],
                'right_w0': joint_angles[4],
                'right_w1': joint_angles[5],
                'right_w2': joint_angles[6],
            }
        else:
            cmd = {
                'left_s0': joint_angles[0],
                'left_s1': joint_angles[1],
                'left_e0': joint_angles[2],
                'left_e1': joint_angles[3],
                'left_w0': joint_angles[4],
                'left_w1': joint_angles[5],
                'left_w2': joint_angles[6],
            }

        arm.set_joint_positions(cmd)
        return True
    else:
        return False

def ik_pykdl_joint(kin, pose, side = "right"):
    position = pose.pose.position
    orientation = pose.pose.orientation
    pos = [position.x,position.y,position.z]
    rot = [orientation.x,orientation.y,orientation.z,orientation.w]
    joint_angles = kin.inverse_kinematics(pos,rot)
    if joint_angles:
        if side == "right":
            cmd = {
                'right_s0': joint_angles[0],
                'right_s1': joint_angles[1],
                'right_e0': joint_angles[2],
                'right_e1': joint_angles[3],
                'right_w0': joint_angles[4],
                'right_w1': joint_angles[5],
                'right_w2': joint_angles[6],
            }
        else:
            cmd = {
                'left_s0': joint_angles[0],
                'left_s1': joint_angles[1],
                'left_e0': joint_angles[2],
                'left_e1': joint_angles[3],
                'left_w0': joint_angles[4],
                'left_w1': joint_angles[5],
                'left_w2': joint_angles[6],
            }

        return cmd
    else:
        return {}

def move_to_pose(left_pose = None, right_pose = None, timeout = 2.0):
    start = rospy.get_time()
    left_result = 1
    right_result = 1
    while not rospy.is_shutdown() and (rospy.get_time() - start) < timeout:
        if left_pose != None : left_result = ik(left_pose,"left")
        if right_pose != None : right_result = ik(right_pose, "right")
    return (right_result == 1 and left_result == 1)

def ik_move(side, pub, jacob, control_mode = 'position', target_dx = None, target_dy = None, target_dz = None, x = None, y = None, z = None, timeout= 5,speed = 1.0):

    if side == "right": arm = Limb("right")
    else: arm = Limb("left")

    initial_pose = get_current_pose(arm)
    target_x = initial_pose.pose.position.x
    target_y = initial_pose.pose.position.y
    target_z = initial_pose.pose.position.z
    
    if target_dx != None: target_x += target_dx
    if target_dy != None: target_y += target_dy
    if target_dz != None: target_z += target_dz

    if x != None: target_x = x
    if y != None: target_y = y
    if z != None: target_z = z

    dx = 100
    dy = 100
    dz = 100

    solution_found = 1

    start = rospy.get_time()
    while (abs(dz) > 0.01) and solution_found == 1 and (rospy.get_time() - start) < timeout and not rospy.is_shutdown():
    # while (abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dz) > 0.01) and solution_found == 1 and (rospy.get_time() - start) < timeout:
    #while (abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dz) > 0.01) and i < 5000:
        
        current_pose = get_current_pose(arm,initial_pose)
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_z = current_pose.pose.position.z
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        #vx = np.sign(dx)*min(0.02,abs(dx))
        #vy = np.sign(dy)*min(0.02,abs(dy))
        #vz = np.sign(dz)*min(0.02,abs(dz))
        vx = dx*speed
        vy = dy*speed
        vz = dz*speed
        #print dx, dy, dz
        p = Point()
        p.x = vx * 1e6
        p.y = vy * 1e6
        p.z = vz * 1e6
        pub.publish(p)

        if control_mode is 'velocity': velocity_control(arm, jacob, vx, vy, vz)
        else: position_control(side, current_pose, vx, vy, vz)
        
    return solution_found

def velocity_control(arm, jacob, vx, vy, vz):
    v_end = np.asarray([vx*0.0,vy*1.2,vz*0.0,0.0,0.0,0.0],np.float32)
    v_joint = np.dot(jacob, v_end)
    cmd = {}
    for idx, name in enumerate(arm.joint_names()):
        v = v_joint.item(idx)
        cmd[name] = v 
    # arm.set_joint_velocities(cmd)
    cmd = {'left_w0': 0.0012733238947391513, 'left_w1': -0.0002816292849537642, 'left_w2': -0.0007010771561622622, 'left_e0': -0.0020298280910253535, 'left_e1': -0.26769762351751303, 'left_s0': 1.1543954429705146, 'left_s1': 0.001086070380806923}
    arm.set_joint_torques(cmd)

def position_control(side, current_pose, vx, vy, vz):
    new_pose = update_current_pose(current_pose,vx,vy,vz)
    solution_found = ik(new_pose,side)

def ik_move_one_step(initial_pose, predict_pose, hdr, arm, kin, target_dx = None, target_dy = None, target_dz = None, side = "right"):

    current_pose = get_current_pose(arm,initial_pose)

    if target_dx == None: 
        target_dx = initial_pose.pose.position.x - current_pose.pose.position.x
    else: 
        target_dx = target_dx + predict_pose.pose.position.x - current_pose.pose.position.x
    if target_dy == None: 
        target_dy = initial_pose.pose.position.y - current_pose.pose.position.y
    else: 
        target_dy = target_dy + predict_pose.pose.position.y - current_pose.pose.position.y
    if target_dz == None: 
        target_dz = initial_pose.pose.position.z - current_pose.pose.position.z
    else:
        target_dz = target_dz + predict_pose.pose.position.z - current_pose.pose.position.z


    new_pose = update_current_pose(current_pose,target_dx,target_dy,target_dz)
    solution_found = ik_pykdl(arm, kin, new_pose, side = side)
    return solution_found, new_pose

def listToPose(list):
    pose = PoseStamped()
    pose.header = Header(stamp=rospy.Time.now(), frame_id='base')
    pose.pose.position.x = list[0]
    pose.pose.position.y = list[1]
    pose.pose.position.z = list[2]
    pose.pose.orientation.x = list[3]
    pose.pose.orientation.y = list[4]
    pose.pose.orientation.z = list[5]
    pose.pose.orientation.w = list[6]
    return pose

def get_current_pose(arm,initial_pose = None):
    
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    pose = arm.endpoint_pose()
    while len(pose) == 0:
        print "Error"
        rospy.sleep(1)
        pose = arm.endpoint_pose()
    ep_position = pose['position']
    ep_orientation = pose['orientation']

    if initial_pose == None:
        
        current_pose = PoseStamped(
                    header=hdr,
                    pose=Pose(
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
        )
    else:
        current_pose = copy.deepcopy(initial_pose)
        current_pose.pose.position.x = ep_position.x
        current_pose.pose.position.y = ep_position.y
        current_pose.pose.position.z = ep_position.z

    return current_pose

def update_current_pose(current_pose,dx,dy,dz):
    new_pose = copy.deepcopy(current_pose)
    new_pose.pose.position.x += dx
    new_pose.pose.position.y += dy
    new_pose.pose.position.z += dz
    #print dx,dy,dz
    return new_pose

def rotate(arm, target_rotate_angle, timeout = 2):

    dtheta = 100
    current_angles = arm.joint_angles()
    current_angle = current_angles['right_w2']
    target_angle = current_angle + target_rotate_angle
    cmd = current_angles
    start = rospy.get_time()
    while abs(dtheta) > 0.01 and not rospy.is_shutdown() and (rospy.get_time() - start) < timeout:
        current_angles = arm.joint_angles()
        current_angle = current_angles['right_w2']
        dtheta = target_angle - current_angle
        cmd['right_w2'] = current_angle + dtheta/10.0
        arm.set_joint_positions(cmd)
        rospy.sleep(0.01)

def pour(arm, target_rotate_angle):

    rotate(arm, target_rotate_angle)
    rospy.sleep(2)
    rotate(arm, -target_rotate_angle)

def clean_shutdown():
    print "Demo finished"
    rospy.signal_shutdown("Done")