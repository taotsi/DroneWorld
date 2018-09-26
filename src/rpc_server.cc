#include "rpc_server.h"

void PackServer(rpc::server& server, World& world) {
	server.bind("test", [&world]() {world.test(); });
	server.bind("test_stixel", [&world]() {return world.drone_list_["drone1"]->stixel_->test(); });
	server.bind("GetKde", [&world](int pos) {return world.drone_list_["drone1"]->stixel_->GetKde(pos); });
	server.bind("GetStixel", [&world]() {return world.drone_list_["drone1"]->stixel_->frame_temp[23]; });
}
