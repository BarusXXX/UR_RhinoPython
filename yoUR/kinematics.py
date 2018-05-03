"""
yoUR - Python library for UR robots

This library was initialy developed at ETH Zurich in 2011 at Gramazio Kohler Research.
Since then it was used by students in bachelor, master and MAS levels.
Initial framework was given by Ralph Baertschi, Michael Knauss and Silvan Oesterle.
Considerable contribution was made by Dr. Jason Lim as part of his PhD dissertation 
'YOUR: Robot Programming Tools for Architectural Education' at ETH Zurich in 2016.
This version is used since 2018 at Aalto University in Helsinki and is maintained by Luka Piskorec.

DESCRIPTION

This module contains functions that are used for kinematics
"""


import utils
import Rhino.Geometry as rg
import math

def forward_kinematics(joints, base, dh_parameters):
    """
    Function that returns all the frames in a serial kinematic chain given the joint angles and DH_parameter
    
    Args:
        base: Plane. frame 0 
        joints: List of angles. joint angle in radians
        dh_parameters: Tuple of (joint_distance, joint_angle, link_length, link_twist). This is the Denavit Hartenberg parameter table. 
    
    Returns:
        frames: A list of plane (frames)
    """
    
    _matrices_fk = [utils.dh_matrix(dh) for dh in dh_parameters]
    
    #Set base frame
    frame_0 = rg.Plane.WorldXY
    frame_0.Rotate(joints[0],frame_0.Normal)
    frames_fk = [frame_0]
    _m = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, base)

    #Transform all frames
    for i in range(len(dh_parameters)):
        _p = rg.Plane(frame_0)
        _p.Transform(utils.concatenate_matrices(_matrices_fk[:i + 1]))
        _p.Transform(_m)
        frames_fk.append(_p)
    
    return frames_fk


def inverse_kinematics(target_pose, base, dh_parameters, right_hand = True, wrist_up = False, elbow_up = False):
    """
    Function that returns joint angles given a target_pose. Note this solution is specific to UR at the moment 
    Making calculations in world coordinate system i.e robot base is taken to be world XY
    
    Args:
        base: Plane. robot base
        target_pose: Plane. target frame 
        dh_parameters: Tuple of (joint_distance, joint_angle, link_length, link_twist). This is the Denavit Hartenberg parameter table. 
        right_hand: Boolean. Choose between right or left hand solution
        wrist_up: Boolean. Choose between wrist up or down solution
        elbow_down: Boolean. Choose between elbow up or down solution 
    
    Returns:
        joints: A list of joint angles in radians
    """
    
    _m_target_to_robot = rg.Transform.PlaneToPlane(base, rg.Plane.WorldXY)
    _pose = rg.Plane(target_pose)
    _pose.Transform(_m_target_to_robot)
    
    # ----- 1) Find projection
    frame5_target = _find_penultimate_frame(_pose)
    v_f5 = rg.Vector3d(frame5_target.Origin)
    v_p = rg.Vector3d(frame5_target.Origin.X, frame5_target.Origin.Y, 0)
    
    # ----- 2) Find joint0 (base)
    # find  joint angle 1
    r = dh_parameters[1][0] + dh_parameters[3][0] #66.3 + 43.7 #d2 + d4
    d = v_p.Length
    # include a domain check here for -1 to 1
    # theta angle tangent point - origin - projection pt
    theta_t_o_p = math.pi/2 - math.asin(r/d)  
    v_t = rg.Vector3d(v_p)
    v_t.Unitize()
    v_t *= r
    if right_hand:
        v_t.Rotate(-theta_t_o_p, rg.Vector3d.ZAxis) 
    else:
        v_t.Rotate(theta_t_o_p, rg.Vector3d.ZAxis) 
    v_tp = v_p - v_t
    joint0 = utils.signed_angle(rg.Vector3d.XAxis, v_tp, rg.Vector3d.ZAxis)
    # adjust for handedness option
    if not right_hand:
        joint0 += math.pi
    
    # ----- 3) Find frame 1(shoulder)
    dh_parameters[0][1] += joint0
    t_bs = utils.dh_matrix(dh_parameters[0])
    
    frame1 = rg.Plane.WorldXY
    frame1.Transform(t_bs)
    v_f1 = rg.Vector3d(frame1.Origin)
    
    # ---- 4) Find frame 4
    frame1_t = rg.Plane(frame1)
    frame1_t.Translate(frame1.Normal * (dh_parameters[1][0] + dh_parameters[3][0])) # d2 and d4
    v_f1t = rg.Vector3d(frame1_t.Origin)
    x_line = rg.Intersect.Intersection.PlanePlane(frame5_target,frame1_t)[1]
    
    frame4_z = x_line.Direction
    if wrist_up and frame4_z.Z < 0: 
        frame4_z.Reverse() 
    elif not wrist_up and frame4_z.Z > 0: 
        frame4_z.Reverse() 
    frame4_z.Unitize()
    frame4_z *= dh_parameters[4][0]
    v_f4 = (v_f5 + frame4_z)
    
    # ----- 5) Circle Intersections construction
    cir_shoulder = rg.Circle(frame1_t, dh_parameters[1][2] ) # radius = a2
    frame1_t.Origin = v_f4
    cir_elbow = rg.Circle(frame1_t,392.25)# dh_parameters[2][2]) # radius = a35
    
    xpts = utils.cir_cir_intersection(cir_shoulder, cir_elbow)
    if xpts[0].Z > xpts[1].Z:
         xpts.reverse()
    f2_origin= xpts[1] if elbow_up else xpts[0]
    
    # ----- 6) Find joint1(shoulder) and frame2(elbow)
    v_f2 = rg.Vector3d(f2_origin)
    v_f1t_f2 = v_f2 - v_f1t
    v_f1t_f2.Unitize()
    
    joint1 = utils.signed_angle(-frame1.XAxis, v_f1t_f2, frame1.Normal) #- math.pi/2 #actually comparing -x axis
    dh_parameters[1][1] += joint1
    t_se = utils.dh_matrix(dh_parameters[1])
    frame2 = rg.Plane.WorldXY
    frame2.Transform(t_bs * t_se)
    
    # ----- 7) Find joint2(elbow) and frame3(w1)
    frame2_t = rg.Plane(frame2)
    frame2_t.Translate(-frame2.Normal * dh_parameters[3][0]) #  d4
    v_f2t = rg.Vector3d(frame2_t.Origin)
    v_f2t_f4 = v_f4 - v_f2
    v_f2t_f4.Unitize()
    
    # find signed angle : note comparing x axis
    joint2 = utils.signed_angle(frame2.XAxis, v_f2t_f4, frame2.Normal)
    dh_parameters[2][1] += joint2
    t_ew1 = utils.dh_matrix(dh_parameters[2])
    frame3 = rg.Plane.WorldXY
    frame3.Transform(t_bs * t_se * t_ew1)
    
    # ----- 8) Find joint3(w1) and frame4(w2)
    frame4_z.Unitize()
    joint3 = utils.signed_angle(frame3.YAxis, -frame4_z, frame3.Normal) 
    dh_parameters[3][1] += joint3
    t_w1w2 = utils.dh_matrix(dh_parameters[3])
    frame4 = rg.Plane.WorldXY
    frame4.Transform(t_bs * t_se * t_ew1*t_w1w2)
    
    # ----- 9) Find joint4(w2) and frame5(w3)
    joint4 = utils.signed_angle(-frame4.YAxis, _pose.Normal, frame4.Normal)
    dh_parameters[4][1] += joint4
    t_w2w3 = utils.dh_matrix(dh_parameters[4])
    frame5 = rg.Plane.WorldXY
    frame5.Transform(t_bs * t_se * t_ew1*t_w1w2 * t_w2w3)
    
    # ----- 10) Find joint5(w3) and frame6(tip)
    joint5 = utils.signed_angle(frame5.YAxis, _pose.YAxis, frame5.Normal)
    dh_parameters[5][1] += joint5
    t_w3t = utils.dh_matrix(dh_parameters[5])
    frame6 = rg.Plane.WorldXY
    frame6.Transform(t_bs * t_se * t_ew1*t_w1w2 * t_w2w3 * t_w3t)
    
    return [joint0,joint1,joint2,joint3,joint4,joint5]

def _find_penultimate_frame(target):
    """ Internal function used to find penultimate frame of manipulator.
    Note: temporary - for use with UR only
    
    Args:
        target: Plane. End/target frame
    
    Returns:
        frame: Plane. Next to last frame
    """
    
    v_offset = target.Normal
    v_offset.Reverse()
    v_offset *= 82.5 #d6

    frame = rg.Plane(target)
    frame.Translate(v_offset) 
    return frame