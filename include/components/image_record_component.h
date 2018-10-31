#pragma once
#include "components/base_component.h"
#include <queue>

namespace droneworld{

class ImageRecordComponent : public BaseComponent {
public:
    ImageRecordComponent();
    ~ImageRecordComponent();
    void Update(double DeltaTime);
    void Begin();
	std::queue<ImageResponse> disparity_retreived_;
private:
    msr::airlib::MultirotorRpcLibClient client_;
    void Record(bool save_as_file);
	void RetreiveFrame();
    std::thread thread_handle_;
};
}