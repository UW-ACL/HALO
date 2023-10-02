import random
import numpy as np
import os
from os.path import exists
import time



max_iter = 100000
max_sites = 7
n_min = 3
i = 0
path_input = os .getcwd() + '\\AirSim\\temp\\percep_to_traj.npy'

# Remove previous files
path_pub_addto2halss = os.getcwd() + '\\AirSim\\temp\\traj_to_percep.npy'
path_sub_halss2addto = os.getcwd() + '\\AirSim\\temp\\percep_to_traj.npy'
os.remove(path_pub_addto2halss) if exists(path_pub_addto2halss) else None
os.remove(path_sub_halss2addto) if exists(path_sub_halss2addto) else None

found_file = False
while i < max_iter:
    # Check to see if there is a file to read
    if exists(path_input):
        found_file = True
        try:
            input = np.load(path_input)
        except(ValueError):
            continue
        landing_site_coords = input[:,0:3]
        size_scores = input[:,3]
        print(size_scores)
        drone_scores = input[:,4]
        density_scores = input[:,5]
        uncertainty_scores = input[:,6]
        prox_score = input[:,7]

        remove = size_scores < 1
        good_sites = 0
        for check in remove:
            if check == False:
                good_sites += 1
        if good_sites < n_min:
            np.save(os. getcwd() + '\\AirSim\\temp\\traj_to_percep.npy', np.array([]))
            print("I am sending back 0 landing sites")
            continue

        output_array = np.zeros((good_sites,4))
        output_array[:,0:3] = landing_site_coords[0:good_sites]
        output_array[:,3] = size_scores[0:good_sites]
        output = np.array([])

        # for idx, check in enumerate(remove):
        #     if check == False:
        #         if output.size == 0:
        #             output = output_array[idx]
        #         else:
        #             output = np.vstack((output, output_array[idx-1]))
                
        np.save(os. getcwd() + '\\AirSim\\temp\\traj_to_percep.npy', output_array)
        print("I am sending back " + str(good_sites) + " landing sites")
    elif found_file == False:
        np.save(os. getcwd() + '\\AirSim\\temp\\traj_to_percep.npy', np.array([]))
        print("I am sending back 0 landing sites (first time)")
    i += 1
    time.sleep(0.5)