# Authors: Chris Hayner, haynec@uw.edu
#          Evelyn Madewell
#          Daniel Broyles

import os
import sys
sys.path.append(os .getcwd())

from os.path import exists

import airsim
from airsim.types import Vector3r
from datetime import datetime

from HALSS.HALSS_utils.point_cloud_to_image import collect_lidar
from HALSS.HALSS_utils.network_utils.model_arch import *
from HALSS.HALSS_utils.network_utils.augment import *
from HALSS.HALSS_utils.seg_utils import *
from HALSS.HALSS_utils.halss_utils import *
from AirSim.sim_utils import get_agl_altitude

import numpy as np
import open3d as o3d

import cv2
import numpy as np
import time

# #####################
# Initialize Airsim
# #####################
client = airsim.MultirotorClient()
client.confirmConnection()
client.simFlushPersistentMarkers()

# #####################
# Flags
# #####################
flags = flags_required()
flags.flag_plot_origin = False
flags.flag_plot_circles = False
flags.flag_show_images = True
flags.flag_save_images = False
flags.flag_timing = False
flags.flag_offset = False
flags.flag_debug = False
flags.flag_save_pointclouds = True
flags.flag_coarse_method = 'geo' # Choose "geo" for inclination-based HD or "nn" for learning-based HD


client.simFlushPersistentMarkers()
path_input = os.getcwd() + '\\AirSim\\temp\\traj_to_percep.npy'

# #####################
# Initialize Parameters
# #####################
params = parameters()
params.x_cell_size_coarse = 4 # Size of cell in x direction fosr Coarse Downsampling
params.y_cell_size_coarse = 4 # Size of cell in y direction for Coarse Downsampling
params.x_cell_size_fine = 0.5 # Size of cell in x direction for Fine Downsampling
params.y_cell_size_fine = 0.5 # Size of cell in y direction for Fine Downsampling
params.alpha = 15 # Maximum allowable inclination angle of surface, in degrees
params.max_sites = 7 # Maximum number of sites to be considered
params.grid_res = 320 # Resolution of grid to be used for segmentation (learning-based HD only)
params.thresh = 0.1 # Threshold for variance-aware safety map threshold (learning-based HD only)
params.num_mc_samples = 30 # Number of Monte Carlo samples to be used for variance-aware safety map (learning-based HD only)
params.media_save_path = os.getcwd() + '\\HALSS\\media\\' # Path to save images

# #####################
# Initialize HALSS Datapacket
# #####################
halss_global = halss_data_packet()
halss_global.num_circles = params.max_sites

halss_global.x_cell_size = params.x_cell_size_coarse # Size of cell in x direction for Coarse Downsampling
halss_global.y_cell_size = params.y_cell_size_coarse # Size of cell in y direction for Coarse Downsampling


max_iter = 1e6 # Maximum number of iterations in main loop
i = 0
while i<max_iter:
  t0_while = time.time()
  # #####################
  # Fill out point cloud initially
  # #####################
  if i==0:
    for j in range(10):
      t0_lidar = time.time()
      halss_global.pcd_new = collect_lidar(client) # Collect new point cloud data
      t1_lidar = time.time()
      if flags.flag_timing:
        print("--> [TIMING: Time to collect lidar: ", t1_lidar-t0_lidar, "]")
      if halss_global.pcd_full.size == 0:
        halss_global.pcd_full = halss_global.pcd_new
      else:
  # #####################
  # Add new points to full point cloud array
  # #####################
        halss_global.pcd_full = np.vstack((halss_global.pcd_full, halss_global.pcd_new)) 
  else:
    t0_lidar = time.time()
    halss_global.pcd_new = collect_lidar(client) # Collect new point cloud data
    halss_global.pcd_full = np.vstack((halss_global.pcd_full, halss_global.pcd_new)) 
    if flags.flag_save_pointclouds:
      np.save(params.media_save_path + "Global_Point_Cloud.npy", halss_global.pcd_full)
    t1_lidar = time.time()
    if flags.flag_timing:
      print("--> [TIMING: Time to collect lidar: ", t1_lidar-t0_lidar, "]")

  print("--> [HALSS: Number of points in point cloud: ", halss_global.pcd_full.shape[0], "]")


  # #####################
  # Get AGL altitude from LiDAR Data
  # #####################
  alt = get_agl_altitude(client, halss_global.pcd_new)


  t0_downsample = time.time()
  halss_global.downsample_pointcloud_dict()
  halss_global.downsample_pointcloud() # Downsample point cloud data
  t1_downsample = time.time()
  if flags.flag_timing:
    print("--> [TIMING: Time to downsample global lidar: ", t1_downsample-t0_downsample, "]")
  print("--> [HALSS: Number of points in culled point cloud: ", halss_global.pcd_culled.shape[0], "]")
  

  # #####################
  # Collect Data Packet from Trajectory Planner
  # #####################
  if exists(path_input): # First Check if a file exist
    try:
      halss_global.traj2percep(path_input)
    except(ValueError):
      continue # If file is being accessed, try again on next iteration
    if len(halss_global.radii_ned) != 0: # Case were the landing site radii need to be updated
      halss_global = multithread_update_radii(halss_global, flags, params)
      #halss_global = update_landing_site_radii(halss_global, params, flags)
      t1_full_set_landing_site = time.time()
      halss_global.center_coord_ned_to_uv()
    elif len(halss_global.radii_ned) == 0: # Case were a full new set of sites are needed
      t0_full_set_landing_site = time.time()

      halss_global = coarse_landing_region_selection(halss_global, flags, params)
      halss_global = multithread_fine_site_selection(halss_global, flags, params)
      #halss_global = fine_landing_site_selection(halss_global, params, flags)

      halss_global.variance_map_vis = cv2.applyColorMap(halss_global.variance_map_vis, cv2.COLORMAP_INFERNO)
      if flags.flag_show_images: 
        cv2.imshow("Global Surface Normal", halss_global.surf_norm)
        cv2.imshow("Uncertainty Map", halss_global.variance_map_vis)
      if flags.flag_save_images:
        cv2.imwrite(params.media_save_path + "Global_Surface_Normal.png", halss_global.surf_norm)
        cv2.imwrite(params.media_save_path + "Global_Uncertainty_Map.png", halss_global.variance_map_vis)
        cv2.imwrite(params.media_save_path + "Global_Safety_Map.png", halss_global.safety_map)
  
      halss_global.center_coord_ned_to_uv()
      t1_full_set_landing_site = time.time()
      if flags.flag_timing:
        print("--> [TIMING: Time to find full set of landing sites: ", t1_full_set_landing_site-t0_full_set_landing_site, "]")
    else:
      print("--> [HALSS: Fucked the bed!]")

    if len(halss_global.radii_ned) > params.max_sites:
      print("--> [Warning!: Number of sites exceeds maximum number of sites!]")
    halss_global.radii_uv = halss_global.radii_ned / halss_global.sf_x
    prep_safety_map = prep_safety_mask(halss_global.safety_map)

    t0_scoring = time.time()
    scores = score_landings(halss_global)
    t1_scoring = time.time()
    if flags.flag_timing:
      print("--> [TIMING: Time to score landing sites: ", t1_scoring-t0_scoring, "]")
    halss_global.percep2traj(scores, alt)

  # Flush previous trajectory overlays
  if flags.flag_plot_origin or flags.flag_plot_circles:
    client.simFlushPersistentMarkers()

  if flags.flag_plot_origin:
    safety_map_circle = cv2.circle(safety_map_circle, (int(halss_global.org_x), int(halss_global.org_y)), 3, (255,0,0), -1)
    client.simPlotPoints([Vector3r(0, 0, -2)], color_rgba=[1,0,0,1], size = 25.0, is_persistent = False, duration=1)
  if flags.flag_show_images:
    safety_map_circle = plotCircles((halss_global.center_coords_uv[:,0], halss_global.center_coords_uv[:,1]), halss_global.radii_uv, prep_safety_map) # Plot sites on safety map
    cv2.imshow("Global Safety Map with Updated Radii", cv2.resize(safety_map_circle, (2*safety_map_circle.shape[1], 2*safety_map_circle.shape[0])))
  if flags.flag_save_images:
    cv2.imwrite(params.media_save_path + "Global_Safety_Map_with_Updated_Radii.png", safety_map_circle)
  cv2.waitKey(1)
  #cv2.destroyAllWindows()
  if flags.flag_plot_circles:
    halss_global.plot_circles_unreal(client)

  i += 1
  tf_while = time.time() - t0_while
  if flags.flag_timing:
    print("--> [TIMING: The full while loop ran at ", 1/tf_while, " Hz]")
  else:
    print("--> [TIMING: The full while loop ran at ", 1/tf_while, " Hz]")

pcd_combined = o3d.geometry.PointCloud()
pcd_combined.points = o3d.utility.Vector3dVector(halss_global.pcd_full)
pcd_combined_array = np.asarray(pcd_combined.points)
o3d.visualization.draw_geometries([pcd_combined])