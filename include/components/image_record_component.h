#pragma once
#include "components/base_component.h"

class ImageRecordComponent : public BaseComponent {
public:
    ImageRecordComponent();
    void Update(double DeltaTime) {};

private:
    msr::airlib::MultirotorRpcLibClient client_;
    void ThreadMain();
    //std::thread thread_{ &ImageRecordComponent::ThreadMain, this };
    //ThreadRaii thread_raii_{ std::thread{&ImageRecordComponent::ThreadMain, this} };
};