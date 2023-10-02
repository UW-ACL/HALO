import airsim
import os
import tempfile
import cv2
import time
import sys
import argparse

"""
A simple script to test all the camera APIs. Change the camera name and whether it's an external camera
Example Settings for external camera -
{
    "SettingsVersion": 1.2,
    "SimMode": "Car",
    "ExternalCameras": {
        "fixed1": {
            "X": 0, "Y": 0, "Z": -5,
            "Pitch": -90, "Roll": 0, "Yaw": 0
        }
    }
}
"""


parser = argparse.ArgumentParser()
parser.add_argument('--CAM_NAME', type=str, default='DemoAerialView', help='Camera name')
args = parser.parse_args()
IS_EXTERNAL_CAM = True



client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Test Camera info
# cam_info = client.simGetCameraInfo(CAM_NAME, external=IS_EXTERNAL_CAM)
# print(cam_info)

while True:
    # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
    rawImage = client.simGetImage(args.CAM_NAME, external=True, image_type=airsim.ImageType.Scene)

    png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
    img = png[:,:,0:3]
    cv2.imshow(args.CAM_NAME, img)

    key = cv2.waitKey(1) & 0xFF
    if (key == 27 or key == ord('q') or key == ord('x')):
        break


