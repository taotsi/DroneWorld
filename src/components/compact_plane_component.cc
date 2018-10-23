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
        // PUSH planes_queue_
        CompactPlane();
        // pillar_cluster_queue_->pop();
    }
}

void CompactPlaneComponent::CompactPlane(){
    auto clusters = pillar_cluster_queue_->front();
    auto n_cluster = clusters.size();
    std::vector<Plane> planes;
    for(auto i=0; i<n_cluster; i++){
        n_pillar = clusters[i].size();
        if(n_pillar >= 3){
            PillarClusterToPlane(clusters[i], planes);
        }else(n_pillar == 2){
            planes.push_back(Plane{clusters[i][0], clusters[i][1]});
        }else{ // n_pillar = 1
            // TODO: deal with alone pillars
        }
        PillarClusterToPlane(clusters[i], planes);
    }
    planes_queue_.push(planes);
}

void CompactPlaneComponent::PillarClusterToPlane(std::vector<Pillar> &cluster, 
    std::vector<Plane> &planes) {
    Line2dFitted line{cluster[0], cluster[1]};
    std::vector<double> dist;
    dist.reserve(n_pillar);
    size_t n_pillar = cluster.size();
    int step_size = 5;
    int start = 0, end = 2;
    Plane plane_temp;
    Point p1_temp, p2_temp;
    while(true){
        if(GetSignedDistIfNecessary(line, cluster, start, end, dist, 0.2)){
            int idx_turnpoint;
            if(CheckoutTurnpoint(dist, idx_turnpoint)){
                plane_temp.Reset();
                p1_temp.Set(cluster[start].x(), 
                    line.EstimateY(cluster[start].x()), cluster[start].z1());
                p2_temp.Set(cluster[idx_turnpoint].x(), 
                    line.EstimateY(cluster[idx_turnpoint].x(), 
                    cluster[idx_turnpoint].z2));
                plane_temp.FromPoints(p1_temp, p2_temp);
                planes.push_back(plane_temp);
                start = idx_turnpoint;
            }
        }else{
            // TODO: FillConcave();
        }
        if(end >= n_pillar-1){
            break;
        }else{
            end += (n_pillar-1-end)%step_size == 0 ? 
                step_size : (n_pillar-1-end)%step_size;
        }
    }
}

/* returns false if it's uneven */
bool CompactPlaneComponent::GetSignedDistIfNecessary(Line2dFitted &line, 
    std::vector<Pillar> &pillars, int start, int end, 
    std::vector<double> clipped_dist, double dist_max=0.2) {
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
    int &idx_turnpoint) {
    size_t n_dist = dist.size();
    size_t idx = 1;
    double dist_temp = 0;
    double sum = 0;
    bool is_found = false;
    while(idx<n_dist-1){
        if(abs(dist[idx+1]) < abs(dist[idx])){
            idx +=2;
            if(abs(dist[idx-1]) < abs(dist[idx])){
                if(dist_temp < abs(dist[idx])){
                    dist_temp = abs(dist[idx]);
                    idx_turnpoint = idx;
                    is_found = true;
                }
            }
        }else{
            idx +=1;
        }
    }
    return is_found;
}

} // namespace droneworld