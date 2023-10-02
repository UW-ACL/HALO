import airsim
import numpy as np
import open3d as o3d
import os
import sys
sys.path.append(os .getcwd())
from HALSS.HALSS_utils.point_cloud_to_image import collect_lidar

client = airsim.MultirotorClient()
client.confirmConnection()

pcd_combined_array = np.array([])
for j in range(50):
    new_pcd_data = collect_lidar(client) # Collect new point cloud data
    if pcd_combined_array.size == 0:
        pcd_combined_array = new_pcd_data
    else:
        pcd_combined_array = np.vstack((pcd_combined_array, new_pcd_data)) 
    
pcd_combined = o3d.geometry.PointCloud()
pcd_combined.points = o3d.utility.Vector3dVector(pcd_combined_array)
pcd_combined_array = np.asarray(pcd_combined.points)
o3d.visualization.draw_geometries([pcd_combined])