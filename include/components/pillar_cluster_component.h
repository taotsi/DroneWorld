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
    std::queue<PillarCluster> pillar_cluster_horizon_queue_;
    std::queue<PillarCluster> pillar_cluster_queue_;
    // temp data
    std::vector<std::vector<Pillar>> temp_pillars;
    // for rpclib server
    std::vector<std::vector<std::vector<double>>> GetPillarClusterHorizon();
    std::vector<std::vector<std::vector<double>>> GetPillarCluster();
private:
    void RunCluster();
    void HorizontalCluster();
    void VerticalCluster();
    void FormPlane();
    void FilterPlane();
};