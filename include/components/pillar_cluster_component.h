#pragma once
#include "components/base_component.h"

class PillarClusterComponent : public BaseComponent {
public:
    PillarClusterComponent(
        std::queue<PillarFrame>* pillar_frame_queue);
    ~PillarClusterComponent();
    void Begin();
    void Update(double DeltaTime);
    /* data */
    std::queue<PillarFrame>* pillar_frame_queue_;
private:
    void RunCluster();
};