# Drone World

[中文](./docs/readme_cn.md)

## Intro

Simulation program for our lab's project, base on Airsim.

## How to build

Put this repo beside the Airsim directory, not inside! Right-click the solution in vs2017 solution explorer, click "Add", find the .vcxproj file for this project and add it. Right-click this project in vs2017 solution explorer, click "Build". Remember to build this project(as well as the whole solution) in x64 mode.

If you were to add or delete .cc or .h files, the easiest way is to do this in the airsim solution. Otherwise you'll have to modify the content in  DroneWorld.vcxproj and the AirSim.sln.

## How to run

First you have to run an Airsim program, download one from [here](https://github.com/Microsoft/AirSim/releases). Then run the built DroneWorld.exe and the python script in py_client/.

### Input Command

After the program finishes initialization, you can input some commands to console, here ther are:

- `exit`

    Exit program.
    
- `ls`
    
    List the drones available right now. Currently there's only one drone, so this command is useless.
    
- `select`
    
    Select the drone you'd like to control. Currently there's only one drone, so this command is useless.
    
- `go x y z`
    
    for example, `go 0 5 2` means flying to (0, 5, 2). ENU coordinate system, +x, +y and +z means east, north, and up.

## Python Client

See all the available Python APIs in py_client/rpc_client.py, and test code in py_client/test.py

### Quick steps

here are the steps to take for python users

- run a airsim simulation program

  which you can download in the Airsim github repo's releases.
 
- run DroneWorld.exe

  which you should build in advance on vs2017(x64, debug or release), find the exe file in build/
  
- run your python scripts

  `python test.py` for example

## TODO

- record on C++
- add noise
- find a window scene
- plane forming
- embed opencv
- thread competition for disparity queue and other queue
