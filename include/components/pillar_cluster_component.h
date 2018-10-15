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
    enum ComplementStatus {
        kJamb,
        kSill,
        kHead
    };
    void RunCluster();
    void Cluster();
    ComplementStatus CompletePillar(Pillar &pillar, 
        double z_max, double z_min, double h_thh, bool is_forcibly=false);
    void FormPlane();
    void FilterPlane();

};