# ..:: Imports ::..
import airsim
import sys
sys.path.append('C:\\Users\\chris\\Documents\\HALO')
from monte_carlo_utils import scoring

# Debugger
from pdb import set_trace as debug

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

scoring(client, 49.54, 62.25, 0.30, 50)