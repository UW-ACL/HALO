import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import random
import time

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

state = client.getMultirotorState()

#airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

state = client.getMultirotorState()

#print("state: %s" % pprint.pformat(state))

#airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
max_iter = 2000
i = 0
client.moveToPositionAsync(0, 0, -100, 15).join()



while i < max_iter:
    rand_num_1 = 2*random.random()-1
    rand_num_2 = 2*random.random()-1
    x_val = 300*rand_num_1
    y_val = 300*rand_num_2
    print("Moving to: %d, %d" % (x_val, y_val))
    client.moveToPositionAsync(x_val, y_val, -100, 20).join()
    i+=1


airsim.wait_key('Press any key to reset to original state')

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
