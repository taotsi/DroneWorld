#include "components/compact_plane_component.h"
#include <iostream>

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
    if(n_cluster > 1){
        for(auto i=0; i<n_cluster; i++){
            PillarClusterToPlane(0.2, 5, pillar_clusters[i], planes_queue_);
        }
    }else{
        std::<< n_cluster << " pillars in this cluster\n";
    }
}

void CompactPlaneComponent::PillarClusterToPlane( std::vector<Pillar> &cluster, 
    std::vector<Plane> &planes) {
    Line2dFitted line{cluster[0], cluster[1]};
    std::vector<double> dist;
    dist.reserve(n_pillar);
    double dist_max = 0;
    size_t n_pillar = cluster.size();
    if(n_pillar>=3){
        
    }else if(n_pillar == 2){
        planes.push_back(Plane{cluster[0], cluster[1]});
    }else{ // n_pillar = 1
        // TODO
    }
}

/* returns false if it's uneven */
bool CompactPlaneComponent::GetSignedDist(Line2dFitted &line, 
    std::vector<Pillar> &pillars, int start, int end, 
    std::vector<double> clipped_dist, int n_flip_max, double dist_max=0.2) {
    if(end-start < 3){
        std::cout << "CompactPlaneComponent::GetSignedDist, end-start<2\n ";
        return false;
    }
    int n_pillar = end - start;
    clipped_dist.reserve(n_pillar);
    clipped_dist.clear();
    int n_flip_max = n_pillar*0.3<1 ? 1 : n_pillar*0.3;
    double dist_temp;
    double dist_1 = line.DistClipped(pillars[start].x(), pillars[start].y());
    int prev_flip_flag = 0, flip_flag = 0;
    if(dist_1 != 0){
        prev_flip_flag = dist_1 > 0 ? 1 : -1;
    }
    int n_flip = 0;
    for(int i=start; i<=end; i++){
        dist_temp = line.DistClipped(pillars[i].x(), pillars[i].y(), 0.1);
        if(dist_temp != 0){
            flip_flag = dist_temp > 0 ? 1 : -1;
        }
        if(flip_flag == -prev_flip_flag){
            n_flip++;
            if(n_flip > n_flip_max){
                return false;
            }
        }
        clipped_dist.push_back(dist_temp);
    }
    return true;
}

bool CompactPlaneComponent::CheckoutTurnpoint(std::vector<double> dist, 
    double dist_max) {
    size_t n_dist = dist.size();
    size_t idx = 1, idx_max=0;
    double dist_temp = 0;
    while(idx<n_dist-1){
        if(abs(dist[idx+1]) < abs(dist[idx])){
            idx +=2;
            if(abs(dist[idx-1]) < abs(dist[idx])){
                if(dist_temp < abs(dist[idx])){
                    dist_temp = abs(dist[idx]);
                    idx_max = idx;
                }
            }
        }else{
            idx +=1;
        }
    }
}

} // namespace droneworld