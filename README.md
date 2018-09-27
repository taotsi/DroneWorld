# Drone World

[中文](./docs/readme_cn.md)

## Intro

Simulation program for our lab's project, base on Airsim.

## How to build

Put this repo beside Airsim dir, not inside. Right-click the solution in vs2017 solution explorer, click "Add", find the .vcxproj file for this project and add it. Right-click this project in vs2017 solution explorer, click "Build". Remember to build this project(as well as the whole solution) in x64 mode.

## How to run

run the built .exe and the python script in py_client/.

## Python Client

all you need to checkout are `DroneWorld/src/rpc_server.cc` and `DroneWorld/py_client/rpc_client.py`. Here's a example:

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

#### run a airsim simulation program

 which you can download in the Airsim github repo's releases.
 
#### run DroneWorld executable

  which you should build in advance in vs2017(x64, debug or release), find the exe file in build/
  
#### run your python scripts

  rpc_client.py for example
  code in rpc_client.py is not very elegant, I'll pack them in classes later.

## TODO

- sliding-block filter(object detector)
- thread competition for disparity queue and other queue
- add necessary opencv libs
- expose rpc server api
- pack rpc client api
