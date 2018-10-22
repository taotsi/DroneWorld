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
    if(n_cluster > 1){
        for(auto i=0; i<n_cluster; i++){
            PillarClusterToPlane(0.2, 5, pillar_clusters[i], planes_queue_);
        }
    }else{
        std::<< n_cluster << " pillars in this cluster\n";
    }
}

void CompactPlaneComponent::PillarClusterToPlane(double d_max, int n_flip_max, 
    std::vector<Pillar> &cluster, std::vector<Plane> &planes){
    Line2dFitted line{cluster[0], cluster[1]};
    std::vector<double> dist;
    dist.reserve(n_pillar);
    double dist_max = 0;
    size_t n_pillar = cluster.size();
    int start = 0, end = 2;
    // 1 for bigger and -1 for lesser
    int prev_flip_flag = 0, flip_flag = 0;
    int n_flip;
    while(end < n_pillar){
        dist.clear();
        line.AddPoint(cluster[end].x(), cluster[end].y());
        for(int i=start; i<=end; i++){
            dist.push_back(line.DistFromPoint(cluster[i].x(), cluster[i].y()));
        }
        d_max_temp = std::max_element(dist.begin(), dist.end());
        if(*dist_max > d_max){
            prev_flip_flag = 
                cluster[start].y() > line.EstimateY(cluster[start].x())
                ? 1 : -1;
            for(int i=start+1; i<=end; i++){
                flip_flag = cluster[i].y() > line.EstimateX(cluster[i].x())
                    ? 1 : -1;
                if(flip_flag != prev_flip_flag){
                    n_flip++;
                    if(n_flip > n_flip_max){
                        // TODO:what's next??
                        break;
                    }
                }
                prev_flip_flag = flip_flag;
            }
        }
        end++;
    }
}

} // namespace droneworld