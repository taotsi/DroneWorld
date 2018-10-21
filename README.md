# Drone World

[中文](./docs/readme_cn.md)

## Intro

Simulation program for our lab's project, base on Airsim.

## How to build

Put this repo beside the Airsim directory, not inside! Right-click the solution in vs2017 solution explorer, click "Add", find the .vcxproj file for this project and add it. Right-click this project in vs2017 solution explorer, click "Build". Remember to build this project(as well as the whole solution) in x64 mode.

If you were to add or delete .cc or .h files, the easiest way is to do this in the airsim solution. Otherwise you'll have to modify the content in  DroneWorld.vcxproj and the AirSim.sln.

## How to run

First you have to run an Airsim program, download one from [here](https://github.com/Microsoft/AirSim/releases). Then run the built DroneWorld.exe and the python script in py_client/.

### Input

- `exit`

## How to checkout the code

For start, see for all the code inside `Begin()` and `Update()` in each class. Most work is done in various component classes, checkout them and go to `Begin()` and `Update()` first. Then you'll know what to look at next. The code is a mess, sorry~

## Python Client

If you're insterested in the rpc function binding, checkout `DroneWorld/src/rpc_server.cc` and `DroneWorld/py_client/rpc_client.py`. Here's a example, skip it if you just care about the rpc client api.

rpc_server.cc
```
#include "rpc_server.h"

void PackServer(rpc::server& server, World& world) {
	server.bind("GetKde", [&world](int pos) {return world.drone_list_["drone1"]->stixel_->GetKde(pos); });
}
```
`PackServer()` binds all the functions or data you'll need in python. When you need those functions and data, just call them in python, like this:

rpc_client.py
```
import msgpackrpc
import matplotlib.pyplot as plt

client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 8080))
kde = client.call("GetKde")
plt.plot(kde)
plt.show()
```

### Quick steps

here are the steps to take for python users

- run a airsim simulation program

  which you can download in the Airsim github repo's releases.
 
- run DroneWorld executable

  which you should build in advance on vs2017(x64, debug or release), find the exe file in build/
  
- run your python scripts

  `python test.py` for example

## TODO

- add namespace to all classes
- linear fitting
- record on C++
- message queue for input handling
- copy and move ctors for some data types
- add noise
- find a window scene
- plane forming
- ban component classed copy ctor and move ctor
- embed opencv
- thread competition for disparity queue and other queue
- a thread for input process
