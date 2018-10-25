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
        std::cout << "compact ready\n";
    }else{
        std::cout << "pillar_cluster_queue_ is empty\n";
    }
    if(!planes_queue_.empty()){
        auto &planes = planes_queue_.front();
        if(planes.empty()){
            std::cout << "no planes!\n";
        }
        for(auto &plane : planes){
            plane.Print();
        }
    }else{
        std::cout << "planes_queue_ is empty\n";
    }
}

void CompactPlaneComponent::CompactPlane(){
    auto clusters = pillar_cluster_queue_->front();
    auto n_cluster = clusters.size();
    std::vector<Plane> planes;
    for(auto i=0; i<n_cluster; i++){
        auto n_pillar = clusters[i].size();
        std::cout << "---- cluster " << i << ", " << n_pillar << " pillars\n";
        if(n_pillar >= 3){
            PillarClusterToPlane(clusters[i], planes);
        }else if(n_pillar == 2){
            std::cout << "two pillars\n";
            planes.push_back(Plane{clusters[i][0], clusters[i][1]});
        }else{ // n_pillar = 1
            // TODO: deal with single pillars
            std::cout << "one single pillar or no pillar at all\n";
        }
    }
    planes_queue_.push(planes);
}

void CompactPlaneComponent::PillarClusterToPlane(std::vector<Pillar> &cluster, 
    std::vector<Plane> &planes) {
    Line2dFitted line{cluster[0], cluster[1]};
    std::vector<double> dist;
    int n_pillar = static_cast<int>(cluster.size());
    dist.reserve(n_pillar);
    int step_size = 5;
    int start = 0, end = 2;
    Plane plane_temp;
    Point3D p1_temp, p2_temp;
    while(true){
        std::cout << "\tstart: " << start << ", end: " << end << "\n"; 
        if(GetSignedDistIfNecessary(line, cluster, start, end, dist, 0.5)){
            std::cout << "\t\tGot dist\n";
            int idx_turnpoint;
            if(CheckoutTurnpoint(dist, idx_turnpoint)){
                std::cout << "\t\tGot turnpoint\n";
                auto p1_ye = line.EstimateY(cluster[start].x());
                auto p1_y = p1_ye==0 ? cluster[start].y() : p1_ye;
                p1_temp.Set(cluster[start].x(), p1_y, cluster[start].z1());
                auto p2_ye = line.EstimateY(cluster[idx_turnpoint].x());
                auto p2_y = p2_ye==0 ? cluster[idx_turnpoint].y() : p2_ye;
                p2_temp.Set(cluster[idx_turnpoint].x(), p2_y, 
                    cluster[idx_turnpoint].z2());
                plane_temp.FromPoints(p1_temp, p2_temp);
                planes.push_back(plane_temp);
                start = idx_turnpoint;
            }
        }else{
            // TODO: FillConcave();
            std::cout << "FillConcave() is called\n";
            break;
        }
        if(end < n_pillar-1){
            if(n_pillar-1-end >= step_size){
                end += step_size;
            }else{
                end = n_pillar-1;
            }
        }else{
            std::cout << "\t** loop out of range, push last plane\n";
            line.Print();
            auto p1_ye = line.EstimateY(cluster[start].x());
            auto p1_y = p1_ye==0 ? cluster[start].y() : p1_ye;
            p1_temp.Set(cluster[start].x(), p1_y, cluster[start].z1());
            auto p2_ye = line.EstimateY(cluster[end].x());
            auto p2_y = p2_ye==0 ? cluster[end].y() : p2_ye;
            p2_temp.Set(cluster[end].x(), p2_y, cluster[end].z2());
            plane_temp.FromPoints(p1_temp, p2_temp);
            planes.push_back(plane_temp);
            break;
        }
    }
}

/* returns false if it's uneven */
bool CompactPlaneComponent::GetSignedDistIfNecessary(Line2dFitted &line, 
    std::vector<Pillar> &pillars, int start, int end, 
    std::vector<double> clipped_dist, double dist_epsilon) {
    if(end-start < 2){
        std::cout << "CompactPlaneComponent::GetSignedDist, end-start<2\n ";
        return false;
    }
    int n_pillar = end - start + 1;
    clipped_dist.reserve(n_pillar);
    clipped_dist.clear();
    int n_flip_max = static_cast<int>(n_pillar*0.7<1 ? 1 : n_pillar*0.7);
    double dist_temp;
    double dist_1 = line.DistClipped(pillars[start].x(), pillars[start].y(), 
        dist_epsilon);
    int prev_flip_flag = 0, flip_flag = 0;
    if(dist_1 != 0){
        prev_flip_flag = dist_1 > 0 ? 1 : -1;
    }
    int n_flip = 0;
    for(int i=start; i<=end; i++){
        dist_temp = line.DistClipped(pillars[i].x(), pillars[i].y(), 
            dist_epsilon);
        if(dist_temp != 0){
            flip_flag = dist_temp > 0 ? 1 : -1;
        }
        if((flip_flag == 1 && prev_flip_flag == -1)
            || flip_flag == -1 && prev_flip_flag == 1){
            n_flip++;
            if(n_flip > n_flip_max){
                std::cout << "flipped " << n_flip << " times out of " << n_flip_max << "\n";
                return false;
            }
        }
        clipped_dist.push_back(dist_temp);
    }
    return true;
}

bool CompactPlaneComponent::CheckoutTurnpoint(std::vector<double> dist, 
    int &idx_turnpoint) {
    int n_dist = static_cast<int>(dist.size());
    int idx = 1;
    double dist_temp = 0;
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

// for rpc server
std::vector<std::vector<std::vector<double>>> 
CompactPlaneComponent::GetPlanes(){
    if(!planes_queue_.empty()){
        std::vector<std::vector<std::vector<double>>> temp_planes;
        auto &planes = planes_queue_.front();
        for(auto &plane : planes){
            temp_planes.push_back(plane.GetCoor());
        }
        return temp_planes;
    }else{
        std::cout << "planes_queue_ is empty\n";
        return std::vector<std::vector<std::vector<double>>>{};
    }
}

} // namespace droneworld