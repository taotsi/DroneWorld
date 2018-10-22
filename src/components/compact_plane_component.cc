#include "components/compact_plane_component.h"

namespace droneworld{

CompactPlaneComponent::CompactPlaneComponent(
    std::queue<std::vector<std::vector<Pillar>>>* 
    pillar_cluster_queue)
    : pillar_cluster_queue_(pillar_cluster_queue){
    
}

CompactPlaneComponent::~CompactPlaneComponent(){
    
}

void CompactPlaneComponent::Begin(){
    RunCompactPlane();
}

void CompactPlaneComponent::Update(double DeltaTime){
    
}

void CompactPlaneComponent::RunCompactPlane(){
    if(!pillar_cluster_queue_->empty()){
        CompactPlane();
        // pillar_cluster_queue_->pop();
    }
}

void CompactPlaneComponent::CompactPlane(){
    auto pillar_clusters = pillar_cluster_queue_->front();
    auto n_cluster = pillar_clusters.size();
    for(auto i=0; i<n_cluster; i++){
        PillarClusterToPlane(pillar_clusters[i], planes_queue_);
    }
}

void CompactPlaneComponent::PillarClusterToPlane(
    std::vector<Pillar> &cluster, std::vector<Plane> &planes){
    
}

} // namespace droneworld