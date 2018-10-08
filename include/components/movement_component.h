#pragma once
#include "components/base_component.h"

class MovementComponent : public BaseComponent {
public:
    MovementComponent();
    void Begin();
private:
    msr::airlib::MultirotorRpcLibClient client_;
    void MoveTest();
    void BehaveForcibly();
    // TODO:return a enum type to indicate busy or not
    bool BehaveSafely();
};