#pragma once
#include "components/base_component.h"

namespace droneworld{

class MovementComponent : public BaseComponent {
public:
    MovementComponent();
    ~MovementComponent();
    void Begin();
    void Update(double DeltaTime);
    void AddPathPoint(const std::vector<float> &point);
private:
    msr::airlib::MultirotorRpcLibClient client_;
    std::queue<std::vector<float>> path_to_go_;
    std::thread move_thread_;
    void MoveTest();
    void MoveThreadMain();
    void ResetPath(const std::queue<std::vector<float>> &new_path);
};
}