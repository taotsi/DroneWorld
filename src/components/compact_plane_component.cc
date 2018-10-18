#include "components/compact_plane_component.h"

CompactPlaneComponent::CompactPlaneComponent(
    std::queue<std::vector<std::vector<Pillar>>>* 
    pillar_cluster_queue)
    : pillar_cluster_queue_(pillar_cluster_queue){
        
}

CompactPlaneComponent::~CompactPlaneComponent(){
    
}

CompactPlaneComponent::Begin(){
    RunCompact();
}

CompactPlaneComponent::Update(double DeltaTime){
    
}

CompactPlaneComponent::RunCompact(){
    
}