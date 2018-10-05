#include "components/pillar_cluster_component.h"

PillarClusterComponent::PillarClusterComponent(
    std::queue<PillarFrame>* pillar_frame_queue)
    : pillar_frame_queue_(pillar_frame_queue) {
        
}

PillarClusterComponent::~PillarClusterComponent(){
    
}

void PillarClusterComponent::Begin(){
    RunCluster();
}

void PillarClusterComponent::Update(double DeltaTime){
    
}

void PillarClusterComponent::RunCluster(){
    if(!pillar_frame_queue_->empty()){
        // PUSH pillar_cluster_horizon_queue_
        HorizontalCluster();
        pillar_frame_queue_->pop();
    }
    std::cout << "ready\n";
}

void PillarClusterComponent::HorizontalCluster(){
    auto &pillar_frame = pillar_frame_queue_->front();
    PillarClusterHorizon pillar_cluster_horizon;
    for(auto i=0; i<pillar_frame.size(); i++){
        pillar_cluster_horizon.BringIn(pillar_frame[i], 1.0);
    }
    pillar_cluster_horizon_queue_.push(pillar_cluster_horizon);
}