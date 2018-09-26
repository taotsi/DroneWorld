# Drone World

## Intro

Simulation program for our lab's project, base on Airsim.

Put this repo beside Airsim dir, not inside. Then add this project to the AirSim solution.

## About Python Client

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

here are the steps to take for python users:

- run a airsim simulation program
  which you can download in the Airsim github repo's releases.
- run DroneWorld executable
  which you should build in advance in vs2017(x64, debug or release), find the exe file in build/
- run your python programs
  rpc_client.py for example
  code in rpc_client.py is not very elegant, I'll pack them in classes later.

## TODO

- thread competition for disparity queue and other queue
- add necessary opencv libs
- expose rpc server api
- pack rpc client api
