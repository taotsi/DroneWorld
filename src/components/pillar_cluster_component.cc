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
        //pillar_frame_queue_->pop();
    }
    std::cout << "cluster ready\n";
}

void PillarClusterComponent::HorizontalCluster(){
    auto &pillar_frame = pillar_frame_queue_->front();
    PillarClusterHorizon pillar_cluster_horizon;
    for(auto i=0; i<pillar_frame.size(); i++){
        pillar_cluster_horizon.BringIn(pillar_frame[i], 1.0);
    }
    /*
    int s = pillar_cluster_horizon.size();
    for(int i=0; i<s; i++){
        std::cout << "--------------------- cluster " << i+1 
            << " of " << s << "----------------------\n";
        for(auto itr : pillar_cluster_horizon[i]){
            itr.Print();
        }
    }*/
    pillar_cluster_horizon_queue_.push(pillar_cluster_horizon);
}

/* for rpclib server */
std::vector<std::vector<std::vector<double>>>
PillarClusterComponent::GetPillarCluster(){
    std::vector<std::vector<std::vector<double>>> result;
    std::vector<std::vector<double>> cluster;
    cluster.reserve(50);
    if(!pillar_cluster_horizon_queue_.empty()){
        auto &pillar_cluster = pillar_cluster_horizon_queue_.front();
        for(int i=0; i<pillar_cluster.size(); i++){
            cluster.clear();
            for(auto j=0; j<pillar_cluster[i].size(); j++){
                cluster.push_back(pillar_cluster[i][j].GetCoor());
            }
            result.push_back(cluster);
        }
        //std::cout << result.size() << "  " << result[0].size() << "\n";
        return result;
    }else{
        std::cout << "pillar_cluster_horizon_queue_ is empty\n";
        return std::vector<std::vector<std::vector<double>>>();
    }
}
