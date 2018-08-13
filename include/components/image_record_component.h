#pragma once
#include "components/base_component.h"

class ImageRecordComponent : public BaseComponent {
public:
    ImageRecordComponent();
    ~ImageRecordComponent();
    void Update(double DeltaTime) {};
    void Begin();

private:
    msr::airlib::MultirotorRpcLibClient client_;
    void Record();
    void Behave();
    std::thread thread_handle_;
    //std::thread thread_{ &ImageRecordComponent::ThreadMain, this };
    //ThreadRaii thread_raii_{ std::thread{&ImageRecordComponent::ThreadMain, this} };
};