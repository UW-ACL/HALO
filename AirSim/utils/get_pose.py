import airsim
client = airsim.VehicleClient()
pose = client.simGetVehiclePose()
print(pose.position)