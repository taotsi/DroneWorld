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
    RunCompact();
}

void CompactPlaneComponent::Update(double DeltaTime){
    
}

void CompactPlaneComponent::RunCompact(){
    
}
}