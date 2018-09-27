#include "rpc_server.h"

void PackServer(rpc::server& server, World& world) {
	server.bind("GetKde", [&world]() {return world.drone_list_["drone1"]->stixel_->GetKde(); });
}
