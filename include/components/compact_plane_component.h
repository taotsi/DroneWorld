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
    void PillarClusterToPlane(std::vector<Pillar> &cluster, 
        std::vector<Plane> &planes);
    bool GetSignedDistIfNecessary(Line2dFitted &line, std::vector<Pillar> &pillars, 
        int start, int end, std::vector<double> clipped_dist, double dist_max=0.2);
    bool CheckoutTurnpoint(std::vector<double> dist, int &idx_turnpoint);
    void FillConcave();
};
}