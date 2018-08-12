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

#include <thread>
#include "utility.h"

class BaseComponent {
public:
    BaseComponent();
    virtual ~BaseComponent();
    virtual void Update(double DeltaTime) {};
    virtual void Begin();
    void Start() { isOn_ = true; };
    void Stop() { isOn_ = false; };
protected:
    bool isOn_ = false;
    bool isBusy_ = false;
private:
};