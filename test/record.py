"""
put this file in AirSim\PythonClient to run it.
"""
import setup_path
import airsim
import os
import pprint
import tempfile
from threading import Thread
from time import sleep


def State2String(state):
    position = \
        '{:.3f}'.format(state.kinematics_estimated.position.x_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.position.y_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.position.z_val)

    linear_vel = \
        '{:.3f}'.format(state.kinematics_estimated.linear_velocity.x_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.linear_velocity.y_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.linear_velocity.z_val)

    angular_vel = \
        '{:.3f}'.format(state.kinematics_estimated.angular_velocity.x_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.angular_velocity.y_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.angular_velocity.z_val)

    orientation = \
        '{:.3f}'.format(state.kinematics_estimated.orientation.w_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.orientation.x_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.orientation.y_val) + ' '\
        + '{:.3f}'.format(state.kinematics_estimated.orientation.z_val)

    state = position + ' ' + linear_vel + ' ' + angular_vel + ' ' + orientation
    return state


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


def Record(client):
    request = [airsim.ImageRequest("1", airsim.ImageType.DepthPlanner, True),
               airsim.ImageRequest("1", airsim.ImageType.Scene)]
    idx = 0
    print("start recording")
    while(flying):
        responses = client.simGetImages(request)
        state = client.getMultirotorState()
        state = State2String(state)
        SaveFile(responses, idx, state)
        idx += 1
        sleep(0.05)
    print("finish recording")


if __name__ == "__main__":
    global flying
    flying = True

    clientR = airsim.MultirotorClient()
    clientR.confirmConnection()
    client = airsim.MultirotorClient()
    client.confirmConnection()

    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    thread = Thread(target=Record, args=(clientR,))
    thread.start()

    speed = 3
    z = -40
    path_city = [airsim.Vector3r(0, 0, z), airsim.Vector3r(20, 0, z),
                 airsim.Vector3r(55, 35, z), airsim.Vector3r(60, 80, z),
                 airsim.Vector3r(55, 90, z), airsim.Vector3r(45, 95, z),
                 airsim.Vector3r(0, 95, z), airsim.Vector3r(-20, 90, z),
                 airsim.Vector3r(-30, 80, z), airsim.Vector3r(-40, 60, z),
                 airsim.Vector3r(-45, 15, z), airsim.Vector3r(-30, 0, z),
                 airsim.Vector3r(0, 0, z)]

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

    client.moveOnPathAsync(
        path_maze, speedm, 120, airsim.DrivetrainType.ForwardOnly,
        airsim.YawMode(False, 0), -1, 0).join()

    # pos = client.getMultirotorState().kinematics_estimated.position
    # print("(", pos.x_val, ", ", pos.y_val, ", ", pos.z_val, ")")

    # client.moveToPositionAsync(0, 0, -1, 1).join()
    # client.landAsync().join()
    # client.armDisarm(False)
    # client.enableApiControl(False)

    airsim.wait_key("pakt reset")
    flying = False

    client.reset()
    client.enableApiControl(False)
