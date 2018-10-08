"""
put this file in AirSim\PythonClient\multirotor, and add it to PythonClient.pyproj(so as to use the vs2017 intellisense)
"""

import setup_path
import airsim
import os
import pprint
import tempfile
from threading import Thread
from time import sleep
import numpy as np


def StateToString(position, quaternion):
    pos = \
        '{:.3f}'.format(position.x_val) + ' '\
        + '{:.3f}'.format(position.y_val) + ' '\
        + '{:.3f}'.format(position.z_val)
    quat = \
        '{:.3f}'.format(quaternion.w_val) + ' '\
        + '{:.3f}'.format(quaternion.x_val) + ' '\
        + '{:.3f}'.format(quaternion.y_val) + ' '\
        + '{:.3f}'.format(quaternion.z_val)
    return pos + ' ' + quat


def SaveFile(responses, idx, state_str, dir='data'):
    dir_depth = dir + "/Depth"
    dir_image = dir + "/Image"
    try:
        os.makedirs(dir)
        os.makedirs(dir_depth)
        os.makedirs(dir_image)
    except OSError:
        if not (os.path.isdir(dir) and os.path.isdir(dir_depth)
                and os.path.isdir(dir_image)):
            raise

    for j, response in enumerate(responses):
        filename_depth = os.path.join(dir_depth, str(idx))
        filename_image = os.path.join(dir_image, str(idx))

        if response.pixels_as_float:
            print("Type %d, size %d" %
                  (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(
                filename_depth + '.pfm'), airsim.get_pfm_array(response))
        elif response.compress:  # png format
            print("Type %d, size %d" %
                  (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(
                filename_image + '.png'), response.image_data_uint8)
        else:
            print("TODO: uncompressed image")
    # TODO:not sync
    with open(os.path.join(dir, 'state.txt'), 'a') as f:
        f.write(state_str + '\n')


def Record(client, path):
    request = [airsim.ImageRequest("1", airsim.ImageType.DisparityNormalized, True),
               airsim.ImageRequest("1", airsim.ImageType.Scene)]
    idx = 0
    print("start recording")
    while(flying):
        responses = client.simGetImages(request)
        state = StateToString(
            responses[0].camera_position, responses[0].camera_orientation)
        SaveFile(responses, idx, state, path)
        idx += 1
        sleep(0.5)
    print("finish recording")


def RecordMaze():
    clientR = airsim.MultirotorClient()
    clientR.confirmConnection()
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    thread = Thread(target=Record, args=(clientR, "E:/airsim_data_maze", ))
    thread.start()
    speedm = 1
    zm = -1.5
    path_maze = [airsim.Vector3r(0, 0, zm), airsim.Vector3r(0, -8, zm),
                 airsim.Vector3r(-28, -8, zm), airsim.Vector3r(-28, -16, zm),
                 airsim.Vector3r(0, -16, zm), airsim.Vector3r(0, -24, zm),
                 airsim.Vector3r(-38, -24, zm), airsim.Vector3r(-38, 0, zm),
                 airsim.Vector3r(-46, 0, zm), airsim.Vector3r(-46, -10, zm),
                 airsim.Vector3r(-52, -10, zm), airsim.Vector3r(-56, -5, zm),
                 airsim.Vector3r(-58, -5, zm), airsim.Vector3r(-62, -10, zm),
                 airsim.Vector3r(-62, -40, zm), airsim.Vector3r(-58, -48, zm),
                 airsim.Vector3r(0, -48, zm)]
    path_maze_0 = [airsim.Vector3r(0, 0, zm), airsim.Vector3r(0, -8, zm),
                   airsim.Vector3r(-10, -8, zm)]
    client.moveOnPathAsync(
        path_maze_0, speedm, 300, airsim.DrivetrainType.ForwardOnly,
        airsim.YawMode(False, 0), -1, 0).join()
    flying = False
    airsim.wait_key("press any key to reset")

    client.reset()
    client.enableApiControl(False)


def RecordCity():
    clientR = airsim.MultirotorClient()
    clientR.confirmConnection()
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    speed = 10
    z = -30
    path_city = [airsim.Vector3r(0, 0, z), airsim.Vector3r(20, 0, z),
                 airsim.Vector3r(55, 35, z), airsim.Vector3r(60, 80, z),
                 airsim.Vector3r(55, 90, z), airsim.Vector3r(45, 95, z),
                 airsim.Vector3r(0, 95, z), airsim.Vector3r(-20, 90, z),
                 airsim.Vector3r(-30, 80, z), airsim.Vector3r(-40, 60, z),
                 airsim.Vector3r(-45, 15, z), airsim.Vector3r(-30, 0, z),
                 airsim.Vector3r(0, 0, z)]
    path_city_0 = [airsim.Vector3r(0, 0, z), airsim.Vector3r(20, 0, z),
                   airsim.Vector3r(55, 35, z), airsim.Vector3r(60, 80, z),
                   airsim.Vector3r(55, 90, z), airsim.Vector3r(45, 95, z)]
    path_city_1 = [airsim.Vector3r(1, 0, z), airsim.Vector3r(20, 0, z)]
    path_city_2 = [airsim.Vector3r(220, 0, z)]
    client.moveOnPathAsync(
        path_city_1, speed, 120, airsim.DrivetrainType.ForwardOnly,
        airsim.YawMode(False, 0), -1, 0).join()
    thread = Thread(target=Record, args=(clientR, "E:/airsim_data_city"))
    thread.start()
    client.moveOnPathAsync(
        path_city_2, speed, 120, airsim.DrivetrainType.ForwardOnly,
        airsim.YawMode(False, 0), -1, 0).join()

    # client.moveToPositionAsync(0, 0, -1, 1).join()
    # client.landAsync().join()
    # client.armDisarm(False)
    # client.enableApiControl(False)

    flying = False
    airsim.wait_key("press any key to reset")

    client.reset()
    client.enableApiControl(False)


if __name__ == "__main__":
    global flying
    flying = True
    RecordCity()
