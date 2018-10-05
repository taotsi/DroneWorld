#include "components/pillar_cluster_component.h"

PillarClusterComponent::PillarClusterComponent(
    std::queue<PillarFrame>* pillar_frame_queue)
    : pillar_frame_queue_(pillar_frame_queue) {
        
}

PillarClusterComponent::~PillarClusterComponent(){
    
}

void PillarClusterComponent::Begin(){
    
}

void PillarClusterComponent::Update(double DeltaTime){
    
}

void PillarClusterComponent::RunCluster(){
    if(!pillar_frame_queue_->empty()){
        HorizontalCluster();
        pillar_frame_queue_->pop();
    }
}

void PillarClusterComponent::HorizontalCluster(){
    
}