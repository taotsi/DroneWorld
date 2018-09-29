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
#include "data_type.h"

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;

using namespace droneworld;

class BaseComponent {
public:
    BaseComponent();
    virtual ~BaseComponent();
    virtual void Update(double DeltaTime) {};
    virtual void Begin();
    void Start() { is_on_ = true; };
    void Stop() { is_on_ = false; };
protected:
    bool is_on_ = false;
    bool is_busy_ = false;
private:
};