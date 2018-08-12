#pragma once

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"

#include <map>
#include "components/movement_component.h"
#include "components/image_process_component.h"
#include "components/image_record_component.h"
#include "components/communication_component.h"

class World;
class Drone {
public:
    Drone(std::string name);
    virtual ~Drone();
    static std::set<Drone*> drone_set_;
    void Begin();
    virtual void Update(double DeltaTime);

private:
    std::string name_;
    
    MovementComponent Movement_;
    ImageRecordComponent ImageRecord_;
    ImageProcessComponent ImageProcess_;
    CommunicationComponent Communication_;
};

