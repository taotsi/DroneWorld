#pragma once
#include "components/base_component.h"

namespace droneworld{

class CompactPlaneComponent : public BaseComponent {
public:
    CompactPlaneComponent(
        std::queue<std::vector<std::vector<Pillar>>>* pillar_cluster_queue);
    ~CompactPlaneComponent();
    void Begin();
    void Update(double DeltaTime);
    /* data */
    std::queue<std::vector<std::vector<Pillar>>>* pillar_cluster_queue_;
    std::queue<std::vector<Plane>> planes_queue_;
private:
    /* methods */
    void RunCompactPlane();
    void CompactPlane();
    void PillarClusterToPlane(
        std::vector<Pillar> &cluster, std::vector<Plane> &planes);
};
}