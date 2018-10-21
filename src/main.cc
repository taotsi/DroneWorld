#include "world.h"
#include "Drone.h"
#include "rpc_server.h"

int main() {
    World &world = World::Instance();
	rpc::server server{ 8080 };
	PackServer(server, world);
	server.async_run();
    world.Loop();

    return 0;
}
