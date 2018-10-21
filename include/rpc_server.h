#pragma once

#include <rpc/server.h>
#include "world.h"
#include "Drone.h"

namespace droneworld{
    
class RpcServer{
public:
    RpcServer(){
        PackServer();
        server_.async_run();
    }
    ~RpcServer() {};
    void PackServer(World& world) {
        server_.bind("GetDisparityFrame", [&world]() {
            return world.drone_list_["drone1"]->stixel_->GetDisparityFrame(); 
        });
    	server_.bind("GetKde", [&world]() {
            return world.drone_list_["drone1"]->stixel_->GetKde(); 
        });
        server_.bind("GetPillarFrame", [&world](){
            return world.drone_list_["drone1"]->stixel_->GetPillarFrame();
        });
        server_.bind("GetPillarCluster", [&world](){
            return world.drone_list_["drone1"]->cluster_->GetPillarCluster();
        });
        server_.bind("test", [](){return "hello there~";});
    }
private:
    rpc::server server_{8080};
}

}