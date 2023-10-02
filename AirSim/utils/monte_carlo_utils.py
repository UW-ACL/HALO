# ..:: Imports ::..
import airsim
import sys
import numpy as np
import time
import cv2
sys.path.append('C:\\Users\\chris\\Documents\\HALO')
from HALSS.HALSS_utils.halss_utils import plotCircles_NED

# Debugger
from pdb import set_trace as debug

flag_plot_debog_circles = True
flag_show_gt = True

def get_nearest_black_pixel_to_center_of_img(img):
    # Get the center of the image
    center = (img.shape[0]/2, img.shape[1]/2)
    black_pixels = np.where(img == 0)
    # Get the distance of each pixel to the center
    dist = [np.sqrt((black_pixels[0][k] - center[0])**2 + (black_pixels[1][k] - center[1])**2) for k in range(len(black_pixels[0]))]
    # Get the index of the minimum distance
    min_index = np.argmin(dist)
    min_val = dist[min_index]
    return min_val

def spawn_validation_circle_on_map(client, x, y, z, ned_radius):
    # Spawn a circle on the map
    number_of_lines = 50
    plotCircles_NED(client, number_of_lines, ned_radius, _, x, y, z, ned_radius)
    return


def scoring(client, tc_x, tc_y, tc_z, height_offset):

    pose = client.simGetVehiclePose()
    pose.position.x_val = tc_x
    pose.position.y_val = tc_y
    pose.position.z_val = tc_z - height_offset

    client.simSetVehiclePose(pose, ignore_collision=True)
    time.sleep(0.5)

    client.moveToPositionAsync(tc_x, tc_y, tc_z-height_offset, 25).join()
    client.hoverAsync().join()
    time.sleep(0.5)
    rawSegmentation = client.simGetImage("3", airsim.ImageType.Segmentation)
    rawSurfaceNormal = client.simGetImage("3", airsim.ImageType.SurfaceNormals)
    png_segmentation = cv2.imdecode(airsim.string_to_uint8_array(rawSegmentation), cv2.IMREAD_UNCHANGED)
    png_surface = cv2.imdecode(airsim.string_to_uint8_array(rawSurfaceNormal), cv2.IMREAD_UNCHANGED)
    combo, _, _ =  getCombinedMask(png_surface, png_segmentation)
    
    dist_pixels = get_nearest_black_pixel_to_center_of_img(combo)
    dist_meters = (dist_pixels/combo.shape[0]) * (2*height_offset)
    if flag_plot_debog_circles:
        plotCircles_NED(client, 50, height_offset, None, tc_x, tc_y, tc_z, color=[1,0,0,1]) # Validate camera FOV is 90 degrees
        plotCircles_NED(client, 50, dist_meters, None, tc_x, tc_y, tc_z, color=[0,1,0,1])

    print("Distance to nearest obstacle: ")
    print("   Pixels: ", dist_pixels)
    print("   Meters: ", dist_meters)

    if flag_show_gt:
        cv2.imshow('Ground Truth', combo)
        cv2.circle(combo, (int(combo.shape[0]/2), int(combo.shape[1]/2)), int(dist_pixels), (0,0,125), 1)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return dist_meters

def getCombinedMask(surfNorm, seg):
    surfNorm = cv2.cvtColor(np.array(surfNorm), cv2.COLOR_BGRA2BGR)
 
    surfaceNormal_vector_form = surfNorm.copy()

    # conversion from surface normal to pointing vector: color value / 255 * 2.0 - 1.0
    w,h,channels = surfaceNormal_vector_form.shape
    for i in range(channels):
        for j in range(w):
            for k in range(h):
                color_value = surfNorm[j,k,i]
                temp = int(color_value / 255 * 2.0 - 1.0)
                surfaceNormal_vector_form[j,k,i] = temp*255

    # let's get rid of the x and y components, we only care about z component
    surfaceNormal_vector_form[:,:,1] = 0
    surfaceNormal_vector_form[:,:,2] = 0
    #blurred = surfaceNormal_vector_form
    # filter out small noise
    filtered = cv2.bilateralFilter(surfaceNormal_vector_form,30,150,150)
  
    # blur to get get smoother shapes without gaps
    blurred = cv2.GaussianBlur(filtered,(5,5),5)
    
    # blurred = cv2.GaussianBlur(surfaceNormal_vector_form,(5,5),5)

    # convert to gray for binairzation and thresholding
    gray_mask = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    b, surfaceNormal_mask = cv2.threshold(gray_mask,0,256,cv2.THRESH_BINARY)

    gray_mask = cv2.cvtColor(seg, cv2.COLOR_BGR2GRAY)
    b, segmentation_mask = cv2.threshold(gray_mask,0,256,cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    combined_mask = cv2.bitwise_and(surfaceNormal_mask, segmentation_mask)
    return combined_mask, surfaceNormal_mask, segmentation_mask


