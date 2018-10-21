#include "world.h"
#include "Drone.h"
#include "rpc_server.h"

int main() {
    droneworld::World &world = droneworld::World::Instance();
    droneworld::RpcServer server;
    world.Loop();
    
    return 0;
}
