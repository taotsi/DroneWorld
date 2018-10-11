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
    std::queue<PillarClusters> pillar_cluster_queue_;
    // for rpclib server
    std::vector<std::vector<std::vector<double>>> GetPillarCluster();
private:
    void RunCluster();
    void Cluster();
    void FormPlane();
    void FilterPlane();
};