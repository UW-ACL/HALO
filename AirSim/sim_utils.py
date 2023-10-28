# ..:: Imports ::..
# (TODO: can remove redundant imports)
import airsim
from airsim.types import Vector3r

import os
from pprint import pprint
from datetime import datetime
import time
from numpy import cos, sin
from math import inf
import numpy as np
import numpy.linalg as la
import scipy
import scipy.linalg as sla
import scipy.interpolate as interp
from enum import Enum

# Debugger
from pdb import set_trace as debug

"""
Defines error codes for HALO simulation
"""
class ErrorCodes(Enum):
    UNSPECIFIED = 0 # Default
    SUCCESS = 1 # Successful simulation
    FAILED_HANDSHAKE = 2 # Failed to establish handshake between HALSS and ADDTO
    FAILED_DDTO_UPDATE = 3 # Failed to update DDTO (computational error or nonconvergence of algorithm)
    INFINITE_LOOP = 4 # Infinite DDTO computation loop detected
    SIM_TIMEOUT = 5 # Simulation timeout (landing taking too long)
    EXCEPTION = 9 # Exception thrown in simulation loop

def obtain_target_data_from_HALSS(addto, lander, path_sub, path_pub, verbosity=1, flag_acquire=True):
    """
    Main function for publisher-subscriber communication between HALSS and ADDTO

    Inputs:
        addto: PyJulia interface object to call Julia functions in Adaptive-DDTO
        lander: Lander object
        path_sub: path to HALSS subscriber file
        path_pub: path to HALSS publisher file
        verbosity: verbosity level (0, 1, 2)
        flag_acquire: flag to indicate whether acquiring new targets (True) or updating current targets (False)

    Outputs:
        lander: updated Lander object
        successful_handshake: flag to indicate whether communication handshake was successful (True) or not (False)
    """
    # Send/publish current targs to HALSS
    if flag_acquire:
        output_HALSS = np.array([])
    else:
        output_HALSS = np.zeros((lander.n_targs, 9))
        output_HALSS[:,:3] = enu2cam(lander.rf_targs).T
        output_HALSS[:,3] = lander.R_targs
        output_HALSS[:,4] = lander.R_targs
        output_HALSS[:,5] = lander.p_targs["prox_veh"]
        output_HALSS[:,6] = lander.p_targs["pcd"]
        output_HALSS[:,7] = lander.p_targs["µ_99"]
        output_HALSS[:,8] = lander.p_targs["prox_clust"]
    np.save(path_pub, output_HALSS)

    # Wait for HALSS target update
    successful_handshake = True
    flag = True 
    loop_iter = 0
    halss_taking_time_flag = True
    while flag:
        loop_iter += 1
        time.sleep(0.05)
        try:
            new_HALSS_input = np.load(path_sub)
        except(ValueError):
            continue # If file is being accessed, try again on next iteration

        if loop_iter > 50 and halss_taking_time_flag:
            print("--> [HALSS] Handshake taking longer than expected...") if verbosity >= 1 else None
            halss_taking_time_flag = False

        # Check to make sure the new subscribed input:
        #   - Has identical rf_targs to the previously-published output (only checked when updating, not acquiring, since these will not be naturally be different when acquiring)
        #   - Has different other parameters to the previously-published output
        if not flag_acquire:
            equal_rf_targs = np.array_equal(output_HALSS[:,:3], new_HALSS_input[:lander.n_targs,:3])
            equal_targ_params = np.array_equal(output_HALSS[:,3:], new_HALSS_input[:lander.n_targs,3:])
            if equal_rf_targs and not equal_targ_params:
                flag = False
        else:
            if len(new_HALSS_input[:,1]) == lander.n_targs_max:
                flag = False
            # elif len(new_HALSS_input[:,1]) > 0:
                # print("--> [HALSS] Warning: potential mismatch in maximum target configuration between HALSS and ADDTO!")

        if loop_iter > 300:
            successful_handshake = False
            print("--> [HALSS] Failed to establish handshake between ADDTO and HALSS...") if verbosity >= 1 else None
            return (lander, successful_handshake)

    # Acquired new targs (try in a loop until it works, HALSS may be writing to the file at the same time...)
    while True:
        try:
            targs_HALSS = np.load(path_sub)
            break
        except:
            continue

    # Add targ properties to lander properties
    if flag_acquire:
        addto.reallocate_targ_dims_b(lander)
    rf_targs = cam2enu(targs_HALSS[:,0:3].T)
    for j in range(lander.rf_targs.shape[1]):
        for cmp in range(lander.rf_targs.shape[0]):
            lander.rf_targs[cmp,j] = rf_targs[cmp,j]
        lander.R_targs[j] = targs_HALSS[:,3][j]
        lander.p_targs["prox_veh"][j]   = targs_HALSS[:,5][j]
        lander.p_targs["pcd"][j]        = targs_HALSS[:,6][j]
        lander.p_targs["µ_99"][j]       = targs_HALSS[:,7][j]
        lander.p_targs["prox_clust"][j] = targs_HALSS[:,8][j]
    if flag_acquire:
        addto.sort_des_score_b(lander)

    # Secure handshake
    output_HALSS = np.zeros((lander.n_targs, 4))
    output_HALSS[:,0:3] = enu2cam(lander.rf_targs).T
    output_HALSS[:,3] = lander.R_targs
    np.save(path_pub, output_HALSS)

    return (lander, successful_handshake)

def enu2cam(arr_enu):
    """
    ENU (East-North-Up) to CAMERA coordinate transformation 
    Input: original array of size 3xN
    Output: transformed array of size 3xN
    """
    if arr_enu.shape[0] != 3:
        raise Exception('Invalid element length, must be in R^3')
    
    if arr_enu.ndim == 1:
        arr_enu = arr_enu.reshape((-1,1))
    arr_chris = np.zeros(arr_enu.shape)
    N = arr_enu.shape[1]
    for n_ in range(N):
        arr_chris[0,n_] =  arr_enu[0,n_]
        arr_chris[1,n_] = -arr_enu[1,n_]
        arr_chris[2,n_] = -arr_enu[2,n_]

    return arr_chris

def cam2enu(arr_chris):
    """
    CAMERA to ENU coordinate transformation 
    Input: original array of size 3xN
    Output: transformed array of size 3xN
    """
    if arr_chris.shape[0] != 3:
        raise Exception('Invalid element length, must be in R^3')
    
    if arr_chris.ndim == 1:
        arr_chris = arr_chris.reshape((-1,1))
    arr_enu = np.zeros(arr_chris.shape)
    N = arr_chris.shape[1]
    for n_ in range(N):
        arr_enu[0,n_] =  arr_chris[0,n_]
        arr_enu[1,n_] = -arr_chris[1,n_]
        arr_enu[2,n_] = -arr_chris[2,n_]

    return arr_enu

def interp_traj(t_nodes, pos_nodes, N):
    """
    Uses cubic spline interpolation to interpolate a trajectory from a set of nodes (for visualization purposes only!)

    Inputs:
        t_nodes: time nodes
        pos_nodes: position nodes
        N: number of points to interpolate

    Outputs:
        traj: interpolated trajectory
    """
    cs = interp.CubicSpline(t_nodes, pos_nodes, axis=0)
    t_interp = np.linspace(0, t_nodes[-1], N)
    traj = np.zeros((N,3))
    for n_,t_ in enumerate(t_interp):
        traj[n_,:] = cs(t_).flatten()
    return traj

def quat2rpy(quat):
    """
    Quaternion to roll-pitch-yaw (RPY) Euler angles
    """
    q0,q1,q2,q3 = np.ravel(quat)
    rpy = np.zeros((3,1))
    rpy[0,:] = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
    rpy[1,:] = np.arcsin(2*(q0*q2 - q3*q1))
    rpy[2,:] = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
    return rpy

def enu2ned(arr_enu):
    """
    ENU to NED coordinate transformation 
    Input: original array of size 3xN
    Output: transformed array of size 3xN
    """
    if arr_enu.shape[0] != 3:
        raise Exception('Invalid element length, must be in R^3')
    
    if arr_enu.ndim == 1:
        arr_enu = arr_enu.reshape((-1,1))
    arr_ned = np.zeros(arr_enu.shape)
    N = arr_enu.shape[1]
    for n_ in range(N):
        arr_ned[0,n_] =  arr_enu[1,n_]
        arr_ned[1,n_] =  arr_enu[0,n_]
        arr_ned[2,n_] = -arr_enu[2,n_]

    return arr_ned

def ned2enu(arr_ned):
    """
    NED to ENU coordinate transformation 
    Input: original array of size 3xN
    Output: transformed array of size 3xN
    """
    if arr_ned.shape[0] != 3:
        raise Exception('Invalid element length, must be in R^3')
    
    if arr_ned.ndim == 1:
        arr_ned = arr_ned.reshape((-1,1))
    arr_enu = np.zeros(arr_ned.shape)
    N = arr_ned.shape[1]
    for n_ in range(N):
        arr_enu[0,n_] =  arr_ned[1,n_]
        arr_enu[1,n_] =  arr_ned[0,n_]
        arr_enu[2,n_] = -arr_ned[2,n_]

    return arr_enu

def feedback_control(v_cur, v_des, P_gain):
    """
    Simple P controller for velocity commands

    Inputs:
        v_cur: current velocity
        v_des: desired velocity
        P_gain: proportional gain

    Outputs:
        commanded velocity
    """
    return P_gain * (v_cur - v_des)

def nparray2vector3rlist(array):
    """
    Convert a (Nx3) array to a list of vector3r objects
    """
    N = array.shape[0] 
    list = []
    for n_ in range(N):
        vec = Vector3r(array[n_,0], array[n_,1], array[n_,2])
        list.append(vec)

    return list

def get_agl_altitude(client, new_pcd_data):
    """
    Obtains the AGL altitude of the drone from the HALSS LiDAR data and the drone's current position

    Inputs:
        client: AirSim client object
        new_pcd_data: new point cloud data from HALSS

    Outputs:
        alt: AGL altitude of the drone
    """
    state = client.getMultirotorState()
    x_val = state.kinematics_estimated.position.x_val
    y_val = state.kinematics_estimated.position.y_val
    z_val = state.kinematics_estimated.position.z_val

    new_pcd_data_ned = enu2ned(new_pcd_data.T).T
    drone_vecs = np.array([x_val, y_val, z_val]) - new_pcd_data_ned
    norm_drone_vecs = drone_vecs/np.linalg.norm(drone_vecs, axis=1)[:,None]

    vert_vec = np.array([0,0,-1])
    cos_theta_vec = vert_vec.T @ norm_drone_vecs.T
    theta_vec = np.arccos(cos_theta_vec)*180/np.pi

    idx_min = np.where(theta_vec == theta_vec.min())[0][0]
    alt = np.linalg.norm(drone_vecs[idx_min])
    print("--> [HALSS: Altitude: ", alt, "]")
    return alt