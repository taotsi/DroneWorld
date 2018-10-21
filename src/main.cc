#include "world.h"
#include "Drone.h"
#include "rpc_server.h"

int main() {
    World &world = World::Instance();
    RpcServer server;
    world.Loop();

    return 0;
}
