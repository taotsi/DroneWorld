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
    void AddPath(const std::vector<std::vector<float>> &path);
    void SetSpeed(float speed);
private:
    void MoveTest();
    void MoveThreadMain();
    void ResetPath(const std::queue<std::vector<float>> &new_path);
    /* data */
    msr::airlib::MultirotorRpcLibClient client_;
    std::thread move_thread_;
    float speed_ = 1.5;
    std::queue<std::vector<float>> path_to_go_;
};
}