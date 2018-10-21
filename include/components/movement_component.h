#pragma once
#include "components/base_component.h"

class MovementComponent : public BaseComponent {
public:
    MovementComponent();
    ~MovementComponent();
    void Begin();
    void Update(double DeltaTime);
private:
    msr::airlib::MultirotorRpcLibClient client_;
    std::queue<std::vector<double>> path_to_go_;
    std::thread move_thread_;
    void MoveTest();
    void MoveThreadMain();
    void AddPathPoint(const std::vector<double> &point);
    void ResetPath(const std::queue<std::vector<double>> &new_path);
    void BehaveForcibly();
    bool BehaveSafely();
};