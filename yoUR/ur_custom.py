"""
yoUR - Python library for UR robots

This library was initialy developed at ETH Zurich in 2011 at Gramazio Kohler Research.
Since then it was used by students in bachelor, master and MAS levels.
Initial framework was given by Ralph Baertschi, Michael Knauss and Silvan Oesterle.
Considerable contribution was made by Dr. Jason Lim as part of his PhD dissertation 
'YOUR: Robot Programming Tools for Architectural Education' at ETH Zurich in 2016.
This version is used since 2018 at Aalto University in Helsinki and is maintained by Luka Piskorec.

DESCRIPTION

This module manages custom UR script functions
"""


import ur_standard
import Rhino.Geometry as rg
import utils

# ----- Custom motions -----

def move_local(target_vector, accel, vel):
    """
    Function that returns UR script for a custom local motion. Movement is local in tool coordinate system
    
    Args:
        target_vector:  Rhino.Geometry Vector3d.A vector that describes intended direction of motion (relative to current tool pose)
        accel: tool accel in m/s^2
        vel: tool speed in m/s
    
    Returns:
    script: UR script
    """
    
    # Check acceleration and velocity are non-negative and below a set limit
    accel = ur_standard.MAX_ACCEL if (abs(accel) >ur_standard.MAX_ACCEL) else abs(accel)
    vel = ur_standard.MAX_VELOCITY if (abs(vel) > ur_standard.MAX_VELOCITY) else abs(vel)
    # Format target pose
    _pose_target = [target_vector.X/1000, target_vector.Y/1000,target_vector.Z/1000,0.0,0.0,0.0]
    _pose_fmt = "p[" + ("%.4f,"*6)[:-1]+"]"
    _pose_fmt = _pose_fmt%tuple(_pose_target)
    # Format UR script
    script = ur_standard.get_forward_kin("current_pose")
    script += "target_pose = pose_trans(current_pose, %s)\n"%(_pose_fmt)
    script += "movel(target_pose, a = %.2f, v = %.2f)\n"%(accel,vel)
    
    return script

def orient_local(target_plane,accel,vel):
    """
    Function that returns UR script for orienting the robot tip. Movement is local in tool coordinate system
    
    Args:
        target_plane:  Rhino.Geometry Plane. Target orientation plane (relative to current tool pose).
        accel: tool accel in m/s^2
        vel: tool speed in m/s
    
    Returns:
    script: UR script
    """
    
    # Check acceleration and velocity are non-negative and below a set limit
    accel = ur_standard.MAX_ACCEL if (abs(accel) >ur_standard.MAX_ACCEL) else abs(accel)
    vel = ur_standard.MAX_VELOCITY if (abs(vel) > ur_standard.MAX_VELOCITY) else abs(vel)
    # Format target pose
    _matrix = rg.Transform.PlaneToPlane(rg.Plane.WorldXY,target_plane)
    _axis_angle= utils.matrix_to_axis_angle(_matrix)
    _pose_target = [0, 0, 0,_axis_angle[0], _axis_angle[1], _axis_angle[2]]
    _pose_fmt = "p[" + ("%.4f,"*6)[:-1]+"]"
    _pose_fmt = _pose_fmt%tuple(_pose_target)
    # Format UR script
    script = ur_standard.get_forward_kin("current_pose")
    script += "target_pose = pose_trans(current_pose, %s)\n"%(_pose_fmt)
    script += "movel(target_pose, a = %.2f, v = %.2f)\n"%(accel,vel)
    
    return script

def move_axis(_x,_z):
    """
    Function that returns UR script for moving the axis
    
    Args:
        _x: x position of axis in mm.
        _z: z position of axis in mm.
        
    Returns:
        script: UR script
        
    Note:
        x position limt set as 0 - 750 mm
        z position limit set as 300 - 2600 mm
    """
    
    axX = _x/1000
    if axX < 0:
        axX = 0
    elif axX > 0.75:
        axX = 0.75
    
    axZ = _z/1000
    if axZ < 0.3:
        axZ= 0.3
    elif axZ> 2.6:
        axZ= 2.6
        
    script = "move_linear_axis(%s, %s)\n"%(axX,axZ)
    return script

# ----- Custom Interface actions -----

def send_current_joints(ip_address, port):
    """
    Function that returns UR script for opening a socket and then getting current joint information
    
    Args:
        ip_address:  String. Static ip address of computer (Listener)
        ort: int. Port number to listen at
    
    Returns:
        script: UR script
    """
    
    script = ur_standard.socket_open(ip_address,port)
    script += ur_standard.get_joint_positions("joints")
    script += ur_standard.socket_send_string("joints")
    script += "socket_close()\n"
    return script

def send_current_pose(ip_address, port):
    """
    Function that returns UR script for opening a socket and then getting current pose information
    
    Args:
        ip_address:  String. Static ip address of computer (Listener)
        ort: int. Port number to listen at
    
    Returns:
        script: UR script
    """
    
    script = ur_standard.socket_open(ip_address,port)
    script += ur_standard.get_forward_kin("pose")
    script += ur_standard.socket_send_string("pose")
    script += "socket_close()\n"
    return script

# ----- Custom compound actions -----

def pick_l(plane_tos, accel, vel, io):
    """
    Function that returns UR script for picking with linear movement in tool-space.The last plane given will be the picking position
    
    Args:
        plane_tos:A list of Rhino.Geometry Planes. Each a target plane for calculating pose (in UR base coordinate system)
        accel: tool accel in m/s^2
        vel: tool speed in m/s
        io: io number to set to TRUE
        joints: A list of 6 joint angles (double). 
        
    Returns:
        script: UR script
    """
    script =""
    num_planes = len(plane_tos)
    
    # Move to all the waypoints
    for i in range(num_planes):
        script += ur_standard.move_l(plane_tos[i],accel,vel)
        if i == (num_planes - 1):
            script += ur_standard.set_digital_out(io, True)
            script += ur_standard.sleep(2)
    
    return script

def place_l(plane_tos, accel, vel, io, retract = 10):
    """
    Function that returns UR script for placing with linear movement in tool-space.The last plane given will be the placing position.
    There will be a slight retraction motion after placing in the -z direction of the placing plane
    
    Args:
        plane_tos:A list of Rhino.Geometry Planes. Each a target plane for calculating pose (in UR base coordinate system)
        accel: tool accel in m/s^2
        vel: tool speed in m/s
        io: io number to set to TRUE
        joints: A list of 6 joint angles (double). 
        retract: Distance tool tip retracts. in mm.
        
    Returns:
        script: UR script
    """
    script =""
    num_planes = len(plane_tos)
    
    # Move to all the waypoints
    for i in range(num_planes):
        if i == (num_planes - 1):
            script += ur_standard.move_l(plane_tos[i],accel/3,vel/3)
            script += ur_standard.set_digital_out(io, False)
            script += ur_standard.sleep(2)
        else:
            script += ur_standard.move_l(plane_tos[i],accel,vel)
    
    # Retract after placing
    script += move_local(rg.Vector3d(0,0,-retract),accel/2,vel/2) 
    
    return script

# ----- Utility -----
def check_joints(plane_to):
    script = ""
    # call inverse kin
    script += ur_standard.get_inverse_kin("joints",plane_to)
    #check joints
    script += ""
    return script
