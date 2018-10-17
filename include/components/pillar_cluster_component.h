#pragma once
#include "components/base_component.h"

enum ComplementStatus {
    kJamb,
    kSill,
    kHead
};

class PillarClusterComponent : public BaseComponent {
public:
    PillarClusterComponent(
        std::queue<std::vector<Pillar>>* pillar_frame_queue);
    ~PillarClusterComponent();
    void Begin();
    void Update(double DeltaTime);
    /* data */
    std::queue<std::vector<Pillar>>* pillar_frame_queue_;
    std::queue<PillarClusters> primary_pillar_cluster_queue_;
    std::queue<std::vector<std::vector<Pillar>>> filtered_cluster_queue_;
    // for rpclib server
    std::vector<std::vector<std::vector<double>>> GetPillarCluster();
private:
    void RunCluster();
    void PrimaryCluster();
    ComplementStatus CompletePillar(Pillar &pillar, 
        double z_max, double z_min, double h_thh, bool is_forcibly);
    void FillWindow(std::vector<Pillar> jambs, 
        std::vector<Pillar> heads, std::vector<Pillar> sills, 
        double z_max, double z_min);
    void ComplementCluster(SinglePillarCluster &clst_src, 
        std::vector<std::vector<Pillar>> &clst_dst);
    void ComplementFilter();
    void FormPlane();
    void FilterPlane();

};