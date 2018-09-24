#pragma once
#include "components/base_component.h"
#include <queue>

class ImageRecordComponent : public BaseComponent {
public:
    ImageRecordComponent();
    ~ImageRecordComponent();
    void Update(double DeltaTime) {};
    void Begin();
	std::queue<ImageResponse> disparity_retreived_;
private:
    msr::airlib::MultirotorRpcLibClient client_;
    void Record(bool save_as_file);
    void Behave();
    std::thread thread_handle_;
    //std::thread thread_{ &ImageRecordComponent::ThreadMain, this };
    //ThreadRaii thread_raii_{ std::thread{&ImageRecordComponent::ThreadMain, this} };
};