#include "rpc_server.h"

void PackServer(rpc::server& server, World& world) {
	server.bind("test", [&world]() {world.test(); });
}