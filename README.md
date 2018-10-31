# Drone World

## Intro

Simulation program for our lab's project, base on Airsim.

This is a windows program, trying to build or run it on other platforms might take some efforts.

## How to build

Put this repo **BESIDE** the Airsim directory, not inside! Right-click the solution in vs2017 solution explorer, click "Add", find the .vcxproj file for this project and add it. Right-click this project in vs2017 solution explorer, click "Build". Remember to build this project(as well as the whole solution) in x64 mode.

If you were to add or delete .cc or .h files, the easiest way is to do this in the airsim solution. Otherwise you'll have to modify the content in  DroneWorld.vcxproj and the AirSim.sln.

## How to run

Do operate in such an order:

### Configure settings.json

Copy `conf/settings.json` to `~/Documents/Airsim/`. The program will find settings.json and load it automatically. So **EVERYTIME** you pull the repo, checkout if settings.json has been modified. If so, copy it again.

### Run an AirSim program

you can download one from [here](https://github.com/Microsoft/AirSim/releases). Or you can build one on your own, go to Airsim's repo for more details.

### Run DroneWorld.exe

#### Input Command

After the program finishes initialization, you can input some commands to console, here ther are:

- `exit`
    
- `ls`
    
    List the drones available right now. Currently there's only one drone, so this command is useless.
    
- `select [drone_name]`
    
    Select the drone you'd like to control. Currently there's only one drone, so this command is useless.
    
- `go x y z`
    
    For example, `go 0 5 2` means flying to point (0, 5, 2). ENU coordinate system, +x, +y and +z means east, north, and up.

- `disp [column_index_number]`
        
    `disp 45` for example, prints the 45th column of a diparity frame.

- `kde [column_index_number]`

    `kde 45` for example, prints the 45th column of a kde frame.

For more commands, go to src/world.cc, see the definition of ProcessInput().

### Run test.py

There two parts of this program. The C++ part does the major work, and the Python part serves for visualization. The C++ part will open an RPC server on its initialization. And the Python part can connect to the server. All apis are defined in `py_client/rpc_client`. `py_client/test.py` shows how to use them.


## TODO

- fix bug on program exit
- save data as json
- real-time animation
- FillConcave()
  - cylinders
- performance test
  - precision
  - time
- add comments
- optimization
- record on C++
- add noise
- embed opencv
- thread competition for data queues
