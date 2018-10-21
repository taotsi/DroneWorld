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

#include "components/movement_component.h"
#include "components/image_record_component.h"
#include "components/stixel_component.h"
#include "components/pillar_cluster_component.h"
#include "components/communication_component.h"

class World;
class Drone {
public:
    Drone(std::string name);
    virtual ~Drone();
    void Begin();
    virtual void Update(double DeltaTime);
	/* data */
	std::string name_;
	/* components */
    std::unique_ptr<MovementComponent> movement_;
	std::unique_ptr<ImageRecordComponent> image_record_;
	std::unique_ptr<StixelComponent> stixel_;
    std::unique_ptr<PillarClusterComponent> cluster_;
};

