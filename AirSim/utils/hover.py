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
pose = client.simGetVehiclePose()
pose.position.x_val = 0
pose.position.y_val = 0
pose.position.z_val = -4
client.simSetVehiclePose(pose, ignore_collision=True)
client.moveToPositionAsync(pose.position.x_val, pose.position.y_val, pose.position.z_val, 1).join()

airsim.wait_key('Waiting for user input')
client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
