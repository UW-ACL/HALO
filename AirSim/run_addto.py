# Run script for ADDTO routine in AirSim 
# 
# Author: Samuel Buckner
# See following link for AirSim's client usage and capabilities: https://github.com/Microsoft/AirSim/blob/main/PythonClient/airsim/client.py

# ##############
# Python Imports
# ##############
import airsim
import sys
import logging
from matplotlib import cm
from numpy import cos, sin
from math import inf
from copy import copy
import numpy as np
import time
import os
import subprocess
from os.path import exists
from timeit import default_timer as timer
import traceback

# Custom libraries
sys.path.append(os.getcwd())
from AirSim.utils.airsim_traj_utils import *
from HALSS.HALSS_utils.halss_utils import plotCircles_NED

# Debugger
from pdb import set_trace as debug

print("[Python] external libraries successfully imported")


# #############
# Julia Imports
# #############
# PyJulia Setup
from julia import Julia
julia = Julia()
julia.eval("@eval Main import Base.MainInclude: include")
from julia import Main

# Include desired files from Julia
# TODO: rework ADDTO to Julia package, will clean this up
Main.include("AdaptiveDDTO\\src\\setup.jl")
Main.include("AdaptiveDDTO\\src\\params.jl")
Main.include("AdaptiveDDTO\\src\\utils_control.jl")
Main.include("AdaptiveDDTO\\src\\utils_addto.jl")
Main.include("AdaptiveDDTO\\src\\utils_percep.jl")
Main.include("AdaptiveDDTO\\src\\utils_pyjulia.jl")
Main.include("AdaptiveDDTO\\src\\solve_optimal.jl")
Main.include("AdaptiveDDTO\\src\\solve_ddto.jl")
Main.include("AdaptiveDDTO\\src\\plots_utils.jl")
Main.include("AdaptiveDDTO\\src\\plots_core.jl")
Main.set_fonts()
Main.pygui(False)


# ###############
# Parameters
# ###############
# ..:: Paths ::..
path_pub_addto2halss = os.getcwd() + '\\AirSim\\temp\\traj_to_percep.npy'
path_sub_halss2addto = os.getcwd() + '\\AirSim\\temp\\percep_to_traj.npy'
path_sub_alt_halss2addto = os.getcwd() + '\\AirSim\\temp\percep_to_traj_alt.npy'
path_halss = os.getcwd() + '\\AirSim\\run_halss.py'

# Remove previous files
os.remove(path_pub_addto2halss) if exists(path_pub_addto2halss) else None
os.remove(path_sub_halss2addto) if exists(path_sub_halss2addto) else None

# Simulation parameters
dt_print = .1       # [s] Simulation printing update time-step
dt_sim   = .1       # [s] Simulation time-step
h_cut    = 65.      # [m] Altitude condition to commit to best target (guidance lock condition)
h_term   = 2.       # [m] Altitude condition to terminate descent phase
R_ROI    = 25.      # [m] Radius of the region of interest for randomized targets (if flag_HALSS_enabled = False)
sim_timeout = 200.  # [s] Simulation timeout
waypoint_tol = 3.   # [m] Tolerance to reach a waypoint and continue to the next

# User-set Flags
verbosity             = 1 # Set to 0 for no printing, 1 for minimal printing, 2 for full printing
flag_ADDTO_enabled    = True # If false, will just use vanilla DDTO with no recomputations
flag_HALSS_enabled    = True # If false, will use randomized landing sites, no perception-in-the-loop
flag_realtime         = False # If false, will pause intermittently
flag_viz_addto_trajs  = True # If false, will not plot ADDTO trajectories
flag_viz_record       = False # If false, will not record video
flag_glock_to_landing = True # If false, will not follow guidance lock to landing, but rather end sim there
flag_teleport         = True # If false, will not teleport drone to the initial position
flag_require_input    = True # If false, will not require user input to continue
flag_HALSS_subprocess = False # If false, will not run HALSS in a subprocess (must be run separately)
flag_touchdown        = True # If false, will not require touchdown at end of simulation
flag_display_tracking = False # If false, will not display red point for current tracked waypoint

# Initialize quadcopter guidance object
quad = Main.Lander()

# Visualization parameters 
color_targs = cm.get_cmap('gist_rainbow')(np.linspace(0,1,quad.n_targs_max)).tolist()

# Helper functions
findfirst_eq = lambda cond,list : [i for i,x in enumerate(list) if x == cond][0]

# ..:: Set initial conditions ::..
quad.r0[0] = 0.
quad.r0[1] = 0.
quad.r0[2] = 150.
quad.v0[0] = 0.
quad.v0[1] = 0.
quad.v0[2] = 0.
initial_pos = quad.r0

# ..:: Initialize simulation variables ::..
# Simulation status
sim_cur_iter    = 0
sim_cur_time    = 0.0
sim_cur_state   = np.array([quad.r0, quad.v0]).reshape(-1)
sim_cur_control = np.zeros(quad.m)
sim_cur_AGL_alt = 0.0
sim_num_ddto    = 0 # Number of total DDTO computations
sim_succ_ddto   = 0 # Number of successive DDTO computations
sim_error_code  = ErrorCodes.UNSPECIFIED
sim_exception   = False

# Guidance
guid_cur_ddto         = Main.EmptyDDTOSolution(quad.n_targs) # Most recently-computed DDTO solution
guid_cur_traj         = Main.FailedSolution() # Current guidance solution to track
guid_cur_branches     = []
guid_cur_time         = 0.0 # Current time in guidance solution
guid_cur_waypoint     = sim_cur_state
guid_cur_waypoint_idx = 0 # Current waypoint index in guidance solution
guid_prev_ddto        = Main.EmptyDDTOSolution(quad.n_targs)
guid_defer_targ       = -1 # Next deferred target in consideration (tag number)
guid_defer_state      = sim_cur_state # Branch point state of next deferred target
guid_lock_time        = 0.0 # Time at which guidance lock was activated

# Programmatically-set Flags
flag_update_ddto           = True
flag_log_ddto_results      = False
flag_guid_lock_activated   = False # If set to True, Adaptive-DDTO will be disabled and guidance will fix to the best target at the current time
flag_guid_lock_staged      = False # Stage a guidance lock
flag_guid_lock_commanded   = False
flag_guid_recently_updated = True # Flag to indicate if guidance has been updated recently
flag_descent_complete      = False # Signals the end of the simulation/descent phase
flag_temp                  = True # Temporary flag for debugging

# Other variables
time_last_print = 0.0

# Initialize results storage containers
results_guid_update_branches = []
results_guid_update_trajs    = []
results_guid_update_time     = []
results_sim_time             = []
results_sim_state            = []
results_sim_control          = []
results_targs_radii          = []
results_targs_positions      = []

# ######################
# Client Initialization
# ######################
# ..:: Instantiate client ::..
client = airsim.MultirotorClient()
client.confirmConnection() if verbosity>=1 else 0
client.enableApiControl(True)

# Flush previous trajectory overlays
client.simFlushPersistentMarkers()

# ######################
# Pre-flight Positioning
# ######################
# ..:: Takeoff ::..
print("Taking off...") if verbosity>=1 else 0
client.armDisarm(True)
client.takeoffAsync().join()

if flag_teleport:
    # ..:: Move to desired initial condition ::..
    vel_startup = 25
    p0 = np.ndarray.flatten(enu2ned(quad.r0))
    print(f'Teleporting vehicle to ({p0[0]}, {p0[1]}, {p0[2]}) at {vel_startup} m/s...') if verbosity>=1 else 0

    pose = client.simGetVehiclePose()
    pose.position.x_val = p0[0]
    pose.position.y_val = p0[1]
    pose.position.z_val = p0[2]
    client.simSetVehiclePose(pose, ignore_collision=True)
    client.moveToPositionAsync(p0[0], p0[1], p0[2], vel_startup).join()

else:
    # ..:: Move to desired initial condition ::..
    vel_startup = 25
    p0 = np.ndarray.flatten(enu2ned(quad.r0))
    print(f'Moving vehicle to ({p0[0]}, {p0[1]}, {p0[2]})!') if verbosity>=1 else 0
    client.moveToPositionAsync(p0[0], p0[1], p0[2], vel_startup).join()

np.save(path_pub_addto2halss, np.array([]))
if flag_require_input:
    airsim.wait_key(f'[INPUT]: Press any key to begin landing maneuver')
    
if flag_viz_record:
    print("Beginning recording...") if verbosity>=1 else 0
    client.startRecording()
    time.sleep(5)
    print("Started recording!") if verbosity>=1 else 0

if flag_HALSS_subprocess:
    print("Initializing HALSS as a subprocess...") if verbosity>=1 else 0
    cmd = f'python {path_halss}'
    pHALSS = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=False)
    time.sleep(15) # Give HALSS time to start up
    print("HALSS subprocess initialized!") if verbosity>=1 else 0


# ################################
# Landing Maneuver Simulation Loop
# ################################
print('=== Beginning Landing Maneuver! ===') if verbosity>=1 else 0
print("Time: {:.2f} s, Alt: {:.2f} m, Number of targets: {:n}".format(sim_cur_time, sim_cur_AGL_alt, quad.n_targs)) if verbosity>=1 else 0
try:
    while not flag_descent_complete:

        # Obtain current AGL altitude from HALSS
        if flag_HALSS_enabled:
            sim_cur_AGL_alt = np.load(path_sub_alt_halss2addto)[0]

        # ..:: [GUID] Compute new DDTO solution ::..
        #      (If staged to do so)
        if not flag_ADDTO_enabled and sim_num_ddto > 0:
            flag_update_ddto = False
            
        if flag_update_ddto and not flag_guid_lock_activated and not flag_guid_lock_staged:

            # Obtain (N_max - N_current) new targets from perception stack
            if flag_HALSS_enabled:
                # Uses HALSS
                quad,successful_handshake = obtain_target_data_from_HALSS(Main, quad, path_sub_halss2addto, path_pub_addto2halss, verbosity=verbosity, flag_acquire=True)
                if not successful_handshake:
                    sim_error_code = ErrorCodes.FAILED_HANDSHAKE
                    flag_guid_lock_staged = True
            else:
                # Acquires randomized targets
                Main.acquire_new_targets_b(quad, R_ROI)

            if not flag_guid_lock_staged:
                # Set guidance initial conditions as current sim state
                quad.r0[0] = sim_cur_state[0]
                quad.r0[1] = sim_cur_state[1]
                quad.r0[2] = sim_cur_state[2]
                quad.v0[0] = sim_cur_state[3]
                quad.v0[1] = sim_cur_state[4]
                quad.v0[2] = max(min(sim_cur_state[5], quad.v_max_V), -quad.v_max_V)

                print("---> DEBUGGING [{:.2f} s]:".format(sim_cur_time)) if verbosity>=2 else 0
                Main.dump_vehicle_params(quad) if verbosity>=2 else 0

                # Guidance solving
                try:
                    (guid_cur_opt,guid_cur_ddto) = Main.solve_ddto_stack(quad) # Compute DDTO stack solution
                except Exception as e:
                    print("---> DDTO ERROR [{:.2f} s] -- Error statement below:".format(sim_cur_time)) if verbosity>=1 else 0
                    print(e) if verbosity>=1 else 0
                    print("---> UPDATE [{:.2f} s]: Guidance lock staged [DDTO computation unsuccessful -- contingency activated!]".format(sim_cur_time)) if verbosity>=1 else 0
                    guid_cur_ddto = guid_prev_ddto
                    flag_guid_lock_staged = True
                    sim_error_code = ErrorCodes.FAILED_DDTO
                guid_prev_ddto = copy(guid_cur_ddto)
                sim_num_ddto += 1

                if not flag_guid_lock_staged:
                    guid_cur_traj = Main.extract_trunk_segment(quad, guid_cur_ddto) # Track the trunk of DDTO by default
                    flag_guid_recently_updated = True
                    print("---> UPDATE [{:.2f} s]: DDTO solution successfully recomputed [tracking trunk segment]".format(sim_cur_time)) if verbosity>=1 else 0
        
                    # Get current branches
                    guid_cur_branches = []
                    T_targs = copy(quad.T_targs)
                    for k in range(quad.n_targs):
                        λ_targ = quad.λ_targs[k]
                        rej_idx = findfirst_eq(λ_targ, T_targs)
                        T_targs = np.delete(T_targs,rej_idx)
                        branch_sol = Main.BranchSolution(guid_cur_ddto[k].targ_sols[rej_idx], guid_cur_ddto[k].cost_dd, guid_cur_ddto[k].idx_dd, sim_num_ddto)
                        guid_cur_branches.append(branch_sol)

                    # Parameter updates
                    guid_cur_time = 0.0 # Reset guidance time to zero
                    guid_defer_targ = quad.λ_targs[0]
                    guid_defer_state = np.vstack((guid_cur_ddto[0].targ_sols[0].r[:,guid_cur_ddto[0].idx_dd], guid_cur_ddto[0].targ_sols[0].v[:,guid_cur_ddto[0].idx_dd])).reshape(-1)
                    org_pref_order = quad.λ_targs

                    # If trunk segment has zero length (no deferring could take place),
                    # lock guidance to the best target at the current point in time (last index of last DDTO branch solution)
                    # as a contingency measure
                    if len(guid_cur_traj.t) == 0:
                        print("---> UPDATE [{:.2f} s]: Guidance lock staged [DDTO deferral was not possible -- contingency activated!]".format(sim_cur_time)) if verbosity>=1 else 0
                        flag_guid_lock_staged = True

                # Flag updates
                flag_update_ddto = False
                flag_log_ddto_results = True
                sim_succ_ddto += 1

        else:
            sim_succ_ddto = 0

        # If you have to recompute DDTO three times in a row, lock guidance as a contingency case (likely caught in an infinite loop)
        if sim_succ_ddto >= 3:
            print("---> UPDATE [{:.2f} s]: Guidance lock staged [DDTO infinite loop detected -- contingency activated!]".format(sim_cur_time)) if verbosity>=1 else 0
            flag_guid_lock_staged = True
            sim_error_code = ErrorCodes.INFINITE_LOOP

        # ..:: [GUID] Update DDTO-locked target parameters ::..
        if not flag_guid_lock_activated and not flag_guid_lock_staged:
            if flag_HALSS_enabled:
                # Uses HALSS
                quad,successful_handshake = obtain_target_data_from_HALSS(Main, quad, path_sub_halss2addto, path_pub_addto2halss, verbosity=verbosity, flag_acquire=False)
                if not successful_handshake:
                    sim_error_code = ErrorCodes.FAILED_HANDSHAKE
                    flag_guid_lock_staged = True
            else:
                # Updates randomized targets 
                Main.update_locked_targets_b(quad)


        # ..:: [CHECK] Any unsafe targets? ::..
        if not flag_guid_lock_activated and not flag_guid_lock_staged:
            cur_targs = copy(quad.T_targs)
            for targ in cur_targs:
                targ_idx = findfirst_eq(targ, quad.T_targs)

                # Remove target if unsafe
                if quad.R_targs[targ_idx] <= quad.R_targs_min:
                    print("---> UPDATE [{:.2f} s]: Removing target {:n} [bounding radius below the minimum threshold]".format(sim_cur_time, targ)) if verbosity>=1 else 0
                    Main.remove_ddto_target_b(quad, targ)
                    
                    # If this target was queued for deferral, move to next target for deferral
                    if targ == guid_defer_targ:
                        guid_defer_targ = quad.λ_targs[0] # Add the next target in the queue to consideration for deferral
                        guid_defer_idx  = findfirst_eq(guid_defer_targ, org_pref_order)
                        guid_defer_state = np.vstack((guid_cur_ddto[guid_defer_idx].targ_sols[0].r[:,guid_cur_ddto[guid_defer_idx].idx_dd], guid_cur_ddto[guid_defer_idx].targ_sols[0].v[:,guid_cur_ddto[guid_defer_idx].idx_dd])).reshape(-1)

                # Reached minimum target threshold
                if (quad.n_targs < 2) or (quad.n_targs < quad.n_targs_min):
                    print("---> UPDATE [{:.2f} s]: DDTO recomputation staged [target set count below the minimum threshold]".format(sim_cur_time)) if verbosity>=1 else 0
                    flag_update_ddto = True
                    break


        # ..:: [CHECK] Branch switch decision available? ::..
        #      Determining this based on altitude right now to keep things simple
        #      Can do this since altitude is monotonically decreasing, or at least this is what "should" happen :)
        if not flag_guid_lock_activated and not flag_guid_lock_staged:
            while (sim_cur_state[2] <= guid_defer_state[2]): # Ready to determine switch

                # Determine if we should switch or not
                if Main.switch_decision(quad, guid_defer_targ):
                    print("---> UPDATE [{:.2f} s]: DDTO recomputation staged [chose to defer to target {:n}]".format(sim_cur_time, guid_defer_targ)) if verbosity>=1 else 0

                    # Remove all targets except for switch target (`guid_defer_targ`)
                    other_targs = copy(quad.T_targs)
                    idx_delete = findfirst_eq(guid_defer_targ, other_targs)
                    other_targs = np.delete(other_targs, idx_delete)
                    for targ in other_targs:
                        Main.remove_ddto_target_b(quad, targ)

                    flag_update_ddto = True
                    break

                else:
                    print("---> UPDATE [{:.2f} s]: Removing target {:n} [chose to stay on trunk segment]".format(sim_cur_time, guid_defer_targ)) if verbosity>=1 else 0
                    Main.remove_ddto_target_b(quad, guid_defer_targ) # Remove the target that was in consideration for deferral
                    guid_defer_targ = quad.λ_targs[0] # Add the next target in the queue to consideration for deferral
                    guid_defer_idx  = findfirst_eq(guid_defer_targ, org_pref_order)
                    guid_defer_state = np.vstack((guid_cur_ddto[guid_defer_idx].targ_sols[0].r[:,guid_cur_ddto[guid_defer_idx].idx_dd], guid_cur_ddto[guid_defer_idx].targ_sols[0].v[:,guid_cur_ddto[guid_defer_idx].idx_dd])).reshape(-1)

                # Reached minimum target threshold
                if (quad.n_targs < 2) or (quad.n_targs < quad.n_targs_min):
                    print("---> UPDATE [{:.2f} s]: DDTO recomputation staged [target set count below the minimum threshold]".format(sim_cur_time)) if verbosity>=1 else 0
                    flag_update_ddto = True
                    break

        # ..:: [CHECK] Reached Cutoff Altitude? ::..
        if sim_cur_AGL_alt <= h_cut and not flag_guid_lock_activated and not flag_guid_lock_staged:
            print("---> UPDATE [{:.2f} s]: Guidance lock staged [Cutoff altitude of {:.2f} m reached!]".format(sim_cur_time, h_cut)) if verbosity>=1 else 0
            flag_guid_lock_staged = True
        

        # ..:: [GUID] Lock guidance to best current target if necessary ::..
        if flag_guid_lock_staged:

            # Determine the current "best" target in terms of largest radius
            indx_best = np.argmax(quad.R_targs)
            targ_best = quad.T_targs[indx_best]
            defer_idx_best = findfirst_eq(targ_best, org_pref_order)

            guid_cur_traj = Main.extract_guid_lock_traj(quad, guid_cur_ddto, defer_idx_best+1, targ_best, org_pref_order) # (Add +1 since Julia is 1-indexed and Python is 0-indexed :/)
            flag_guid_recently_updated = True
            guid_defer_targ = targ_best
            guid_cur_time   = 0.0 # Reset guidance time to zero
            flag_guid_lock_activated = True
            flag_guid_lock_staged = False
            print("---> UPDATE [{:.2f} s]: Guidance locked to target {:n}".format(sim_cur_time, guid_defer_targ)) if verbosity>=1 else 0
            print("------> [TARGET INFORMATION] Radius: {:.2f} m, Location: [{:.2f}, {:.2f}, {:.2f}] m".format(quad.R_targs[indx_best], quad.rf_targs[0,indx_best], quad.rf_targs[1,indx_best], quad.rf_targs[2,indx_best])) if verbosity>=1 else 0

            # Remove all targets except for locked target (`guid_defer_targ`)
            other_targs = copy(quad.T_targs)
            idx_delete = findfirst_eq(guid_defer_targ, other_targs)
            other_targs = np.delete(other_targs, idx_delete)
            for targ in other_targs:
                Main.remove_ddto_target_b(quad, targ)

            guid_lock_time = sim_cur_time


        # ..:: [VIZ] Overlay trajectory visualizations ::..
        if flag_viz_addto_trajs:
            
            # Flush previous trajectory overlays
            client.simFlushPersistentMarkers()

            if not flag_guid_lock_activated:
                color_defer = [1,1,1,1]

                # Plot the deferrable segment
                spline_trajectory = interp_traj(guid_cur_traj.t, enu2ned(guid_cur_traj.r).T, 100)
                client.simPlotLineStrip(nparray2vector3rlist(spline_trajectory), color_rgba=color_defer, thickness=10, is_persistent=True)

                # Plot all target/branch pairs
                for j in quad.T_targs:
                    # Get indices
                    targ_idx = findfirst_eq(j, quad.T_targs)
                    pref_idx = findfirst_eq(j, org_pref_order)

                    # Branch
                    branch = guid_cur_branches[pref_idx].sol
                    spline_trajectory = interp_traj(branch.t, enu2ned(branch.r).T, 100)
                    client.simPlotLineStrip(nparray2vector3rlist(spline_trajectory), color_rgba=color_targs[j-1], thickness=35, is_persistent=True)            

                    # Target
                    rf_targ_ned = enu2ned(quad.rf_targs[:,targ_idx]).reshape(-1)
                    plotCircles_NED(client, 50, quad.R_targs[targ_idx], rf_targ_ned[0], rf_targ_ned[1], rf_targ_ned[2]-1, color=color_targs[j-1])

            else:
                color_lock = [0,1,0,1] # Green

                # Plot the guidance-locked trajectory and target
                spline_trajectory = interp_traj(guid_cur_traj.t, enu2ned(guid_cur_traj.r).T, 100)
                client.simPlotLineStrip(nparray2vector3rlist(spline_trajectory), color_rgba=color_lock, thickness=35, is_persistent=True)

                # Target
                targ_idx = 0
                rf_targ_ned = enu2ned(quad.rf_targs[:,targ_idx]).reshape(-1)
                plotCircles_NED(client, 50, quad.R_targs[targ_idx], rf_targ_ned[0], rf_targ_ned[1], rf_targ_ned[2], color=color_lock)

            # Plot a red marker for the currently-tracked waypoint
            if flag_display_tracking:
                client.simPlotPoints(nparray2vector3rlist(enu2ned(guid_cur_waypoint[:3].reshape((-1,1))).T), color_rgba=[1,0,0,1], size=50, is_persistent=True)


        # ..:: [SIM] Command Multirotor ::..
        # Determine waypoint index to track in guidance trajectory
        # OLD METHOD (based on altitude only, not too great for robust tracking)
        # for k in range(len(guid_cur_traj.r[0,:])):
        #     if guid_cur_traj.r[2,k] <= sim_cur_state[2]:
        #         guid_cur_waypoint_idx = k + 1
        #         break

        if flag_guid_recently_updated:
            # If guidance was recently updated, identify the nearest point on the trajectory (+1) as the current waypoint
            guid_cur_waypoint_idx = np.argmin(la.norm(guid_cur_traj.r.T - sim_cur_state[:3], axis=1)) + 1
            flag_guid_recently_updated = False
        else:
            # Determine if you are within distance of waypoint, if so iterate to next
            if la.norm(guid_cur_traj.r[:,guid_cur_waypoint_idx].T - sim_cur_state[:3]) <= waypoint_tol:
                guid_cur_waypoint_idx += 1
                # Don't allow waypoint index to exceed trajectory length
                if guid_cur_waypoint_idx >= len(guid_cur_traj.r[0,:]):
                    guid_cur_waypoint_idx = len(guid_cur_traj.r[0,:])-1
        guid_cur_waypoint = guid_cur_traj.r[:,guid_cur_waypoint_idx].T

        # Build out full trajectory after waypoint for good tracking performance
        cmd_pos_traj = nparray2vector3rlist(enu2ned(guid_cur_traj.r[:,guid_cur_waypoint_idx:]).T)

        # # [Hack] if traj only contains one tracking state, append another state 5 meters (arbitrary value) below it
        # # (AirSim does not properly track one state, needs at least 2)
        if len(cmd_pos_traj) == 1:
            cmd_pos_traj.append(cmd_pos_traj[0])
            cmd_pos_traj[1].z_val += 5 # (addition because in NED frame)

        # Vel command magnitude is average of velocities between previous and current waypoint node 
        cmd_vel_mag = la.norm(np.mean(enu2ned(guid_cur_traj.v[:,(guid_cur_waypoint_idx-1):(guid_cur_waypoint_idx+1)]), axis=1))

        # Unpause if not real-time
        if client.simIsPause():
            client.simPause(False)

        # Command vehicle to chase waypoint 
        # if not flag_guid_lock_activated and not flag_guid_lock_commanded:
        client.moveOnPathAsync(path=cmd_pos_traj, velocity=cmd_vel_mag, lookahead=-1, adaptive_lookahead=1)
        time.sleep(dt_sim)

        # Pause if not real-time 
        # if not flag_realtime:
        if not flag_realtime and flag_update_ddto:
            client.simPause(True)

        # Obtain current vehicle state 
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity

        # Make conversions back to ENU frame
        pos_airsim_cur_enu = ned2enu(np.array([[pos.x_val, pos.y_val, pos.z_val]]).T)
        vel_airsim_cur_enu = ned2enu(np.array([[vel.x_val, vel.y_val, vel.z_val]]).T)

        # Set new values after command
        sim_cur_control = Main.optimal_controller(guid_cur_time, sim_cur_state, guid_cur_traj)
        sim_cur_state   = np.vstack((pos_airsim_cur_enu, vel_airsim_cur_enu)).reshape(-1)
        sim_cur_time   += dt_sim
        guid_cur_time  += dt_sim
        sim_cur_iter   += 1

        # ..:: [SIM] Print status updates ::..
        if (sim_cur_time - time_last_print) >= dt_print:
            if not flag_guid_lock_activated:
                print("Time: {:.2f} s, AGL Alt: {:.2f} m, Number of targets: {:n}, Next deferred target: {:n} (switch altitude: {:.2f} m)".format(sim_cur_time, sim_cur_AGL_alt, quad.n_targs, guid_defer_targ, guid_defer_state[2] - (sim_cur_state[2] - sim_cur_AGL_alt))) if verbosity>=1 else 0
            else:
                print("Time: {:.2f} s, AGL Alt: {:.2f} m, Guidance locked to target {:n}!".format(sim_cur_time, sim_cur_AGL_alt, guid_defer_targ)) if verbosity>=1 else 0
            if flag_display_tracking:
                print("               Tracking from [{:.2f}, {:.2f}, {:.2f}] m towards [{:.2f}, {:.2f}, {:.2f}] m waypoint at a velocity of {:.2f} m/s".format(pos_airsim_cur_enu.flatten()[0], pos_airsim_cur_enu.flatten()[1], pos_airsim_cur_enu.flatten()[2], guid_cur_waypoint.flatten()[0], guid_cur_waypoint.flatten()[1], guid_cur_waypoint.flatten()[2], cmd_vel_mag)) if verbosity>=2 else 0
            time_last_print = sim_cur_time

        # ..:: [SIM] Log data ::..
        # log current sim state
        results_sim_state.append(sim_cur_state)
        results_sim_control.append(sim_cur_control)
        results_sim_time.append(sim_cur_time)

        # log current target radii (if a target index is unallocated, insert Inf)
        sim_cur_radii = np.ones(quad.n_targs_max) * inf
        sim_cur_radii[quad.T_targs-1] = quad.R_targs # Subtract by 1 since Python is 0-indexed
        results_targs_radii.append(sim_cur_radii)

        # log current target positions (if a target index is unallocated, insert Inf)
        sim_cur_targ_pos = np.ones((3,quad.n_targs_max)) * inf
        sim_cur_targ_pos[:,quad.T_targs-1] = quad.rf_targs # Subtract by 1 since Python is 0-indexed
        results_targs_positions.append(sim_cur_targ_pos)

        # log conditional sim results (DDTO)
        if flag_log_ddto_results:
            results_guid_update_trajs.append(guid_cur_traj)
            results_guid_update_branches += guid_cur_branches
            results_guid_update_time.append(sim_cur_time)
            flag_log_ddto_results = False


        # ..:: [SIM] Check termination condition ::..
        if not flag_glock_to_landing:
                break
        
        if sim_cur_AGL_alt <= h_term:
            flag_descent_complete = True
            client.cancelLastTask()
            print("---> UPDATE [{:.2f} s]: Terminal altitude of {:.2f} m reached, exiting ADDTO loop!".format(sim_cur_time, h_term)) if verbosity>=1 else 0

            # Unpause if not real-time
            if client.simIsPause():
                client.simPause(False)

        if sim_cur_time >= sim_timeout:
            sim_error_code = ErrorCodes.SIM_TIMEOUT
            print("---> UPDATE [{:.2f} s]: Simulation timed out...") if verbosity>=1 else 0
            break

except Exception as e:
    client.simPause(False)
    sim_exception = True
    if sim_error_code == ErrorCodes.UNSPECIFIED:
        sim_error_code = ErrorCodes.EXCEPTION
    if flag_HALSS_subprocess:
        pHALSS.kill()
        pHALSS.wait()
    print("Error caught during simulation loop at {:.2f} s\n:".format(sim_cur_time)) if verbosity>=1 else 0
    traceback.print_exc() if verbosity>=1 else 0
    print(repr(e)) if verbosity>=1 else 0
    sys.exit()

if not sim_exception:
    if flag_touchdown and flag_glock_to_landing:
        time.sleep(0.2)
        print('Performing final touchdown!') if verbosity>=1 else 0
        client.landAsync().join()
        time.sleep(5)

    # Shut down HALSS if a subprocess
    if flag_HALSS_subprocess:
        print("Terminate HALSS subprocess...") if verbosity>=1 else 0
        pHALSS.kill()
        pHALSS.wait()
        print("HALSS subprocess terminated!") if verbosity>=1 else 0

    # Flush previous trajectory overlays
    if flag_viz_addto_trajs:
        client.simFlushPersistentMarkers()

    # ..:: Store resulting sim time/state/control into a `Solution` object ::..
    e_z = np.array([0,0,1])
    t_sim = results_sim_time
    r_sim = np.stack(results_sim_state, axis=0)[:,0:3].T
    v_sim = np.stack(results_sim_state, axis=0)[:,3:6].T
    T_sim = np.stack(results_sim_control, axis=0)[:,0:3].T
    S_sim = np.stack(results_sim_control, axis=0)[:,3].T
    T_nrm_sim = np.stack([la.norm(T_sim[:,k]) for k in range(len(t_sim))], axis=0)
    Point_sim = np.stack([np.arccos(np.dot(T_sim[:,k],e_z) / la.norm(T_sim[:,k])) for k in range(len(t_sim))], axis=0)
    cost_sim = np.sum(T_nrm_sim) * dt_sim
    results_sim_sol = Main.Solution(t_sim, r_sim, v_sim, T_sim, S_sim, cost_sim, T_nrm_sim, Point_sim)

    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    pos_airsim_cur_enu = ned2enu(np.array([[pos.x_val, pos.y_val, pos.z_val]]).T).reshape(-1)

    if flag_glock_to_landing:
        term_pos = pos_airsim_cur_enu
    else:
        term_pos = guid_cur_traj.r[:,-1]

else:
    term_pos = -1*np.ones(3)
    cost_sim = inf

if flag_viz_record:
    time.sleep(5)
    client.stopRecording()
    print("Recording finished.")

# #################
# Terminate program
# #################
if flag_require_input:
    airsim.wait_key('[INPUT]: Press any key to reset to original state')
client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
time.sleep(2)

# Final print statement
print("\n=== Simulation Results ===")
print("   Initial location (NED): [{:.2f}, {:.2f}, {:.2f}] m".format(initial_pos[0], initial_pos[1], initial_pos[2]))
print("   Terminal landing location (NED): [{:.2f}, {:.2f}, {:.2f}] m".format(term_pos[0], term_pos[1], term_pos[2]))
print("   Cumulative control effort (thrust magnitude): {:.2f} N-s".format(cost_sim))
print("   Error code: {:n}".format(sim_error_code.value))

# # ##############################
# # Store results & generate plots
# # ##############################

# # Adjust other results as needed
# results_targs_radii = np.stack(results_targs_radii, axis=0).T

# # ..:: Plotting ::..
# Main.plot_addto_parametric_3D_trajectory(quad, results_sim_sol, results_guid_update_branches, results_guid_update_trajs, view_az=-45)
# Main.plot_addto_results_icra2023(quad, results_sim_sol, results_guid_update_branches, results_guid_update_trajs, results_targs_radii, results_guid_update_time, guid_lock_time)

# debug()