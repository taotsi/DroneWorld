#include "components/pillar_cluster_component.h"
#include "kde.h"

extern FilterStatus Filter(
    std::vector<double> &vec, int start, int step, 
    double mean, double min, double max);

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
    if(!pillar_cluster_horizon_queue_.empty()){
        // PUSH pillar_cluster_queue_
        VerticalCluster();
        // pillar_cluster_horizon_queue_.pop();
    }else{
        std::cout << "pillar_cluster_horizon_queue_ is empty\n";
    }
    if(!pillar_cluster_queue_.empty()){
        auto &pc = pillar_cluster_queue_.front();
        std::cout << "pillar cluster size: "<< pc.size() << std::endl;
    }else{
        std::cout << "pillar_cluster_queue_ is empty\n";
    }
    std::cout << "cluster ready\n";
}

void PillarClusterComponent::HorizontalCluster(){
    auto &pillar_frame = pillar_frame_queue_->front();
    PillarCluster pillar_cluster_horizon;
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

void PillarClusterComponent::VerticalCluster(){
    auto &cluster_horizon = pillar_cluster_horizon_queue_.front();
    auto n_cluster_horizon = cluster_horizon.size();
    PillarCluster final_cluster;
    std::vector<std::vector<Pillar>> temp_cluster_z1;
    std::vector<std::vector<Pillar>> temp_cluster_z2;
    std::vector<double> kde_z1, kde_z2;
    std::vector<KdePeak> z1_peaks, z2_peaks;
    int kde_width = 200;
    for(auto i_clst_h=0; i_clst_h<n_cluster_horizon; i_clst_h++){
        std::cout << "------ original cluster " << i_clst_h << " ------\n";
        kde_z1.clear();     kde_z2.clear();
        z1_peaks.clear();   z2_peaks.clear();
        auto &z1_vec = cluster_horizon.z1_vec_;
        auto &z2_vec = cluster_horizon.z2_vec_;
        auto max_z1 = std::max_element(z1_vec.begin(), z1_vec.end());
        auto min_z1 = std::min_element(z1_vec.begin(), z1_vec.end());
        auto max_z2 = std::max_element(z2_vec.begin(), z2_vec.end());
        auto min_z2 = std::min_element(z2_vec.begin(), z2_vec.end());
        kde::RetreiveKde(z1_vec, kde_z1, *max_z1, *min_z1, kde_width);
        kde::RetreiveKdePeak(kde_z1, z1_peaks, 0.707, 0, 0.25, 0.2);
        kde::RetreiveKde(z2_vec, kde_z2, *max_z2, *min_z2, kde_width);
        kde::RetreiveKdePeak(kde_z2, z2_peaks, 0.707, 0, 0.25, 0.2);
        /* cluster for z1 */
        std::vector<Pillar> temp_cluster;
        auto n_peaks_z1 = z1_peaks.size();
        auto n_pillar = cluster_horizon[i_clst_h].size();
        std::cout << "n_pillar size: " << n_pillar << std::endl;
        std::cout << "z1_peaks size: " << n_peaks_z1 << "\n";
        for(auto i_pk_z1=0; i_pk_z1<n_peaks_z1; i_pk_z1++){
            temp_cluster.clear();
            double filter_min = 
                static_cast<double>(z1_peaks[i_pk_z1].window_left_) 
                / static_cast<double>(kde_width) + *min_z1;
            double filter_max = 
                static_cast<double>(z1_peaks[i_pk_z1].window_right_) 
                / static_cast<double>(kde_width) + *min_z1;
            for(auto i_pl=0; i_pl<n_pillar; i_pl++){
                if(cluster_horizon[i_clst_h][i_pl].z1() <= filter_max 
                    && cluster_horizon[i_clst_h][i_pl].z1() >= filter_min){
                    temp_cluster.push_back(cluster_horizon[i_clst_h][i_pl]);
                }
            }
            temp_cluster_z1.push_back(temp_cluster);
        }
        temp_pillars = temp_cluster_z1;
        /* cluster for z2 */
        auto n_peaks_z2 = z2_peaks.size();
        auto n_cluster_z1 = temp_cluster_z1.size();
        std::cout << "temp_cluster_z1 size: " << n_cluster_z1 << std::endl;
        std::cout << "z2_peaks size: " << n_peaks_z2 << std::endl;
        for(auto i_pk_z2=0; i_pk_z2<n_peaks_z2; i_pk_z2++){
            temp_cluster.clear();
            double filter_min = 
                static_cast<double>(z2_peaks[i_pk_z2].window_left_) 
                / static_cast<double>(kde_width) + *min_z2;
            double filter_max = 
                static_cast<double>(z2_peaks[i_pk_z2].window_right_) 
                / static_cast<double>(kde_width) + *min_z2;
            for(auto i_clst_z1=0; i_clst_z1<n_cluster_z1; i_clst_z1++){
                auto n_each_cluster_z1 = temp_cluster_z1[i_clst_z1].size();
                for(auto i_pl=0; i_pl<n_each_cluster_z1; i_pl++){
                    if(temp_cluster_z1[i_clst_z1][i_pl].z2() <= filter_max
                        && temp_cluster_z1[i_clst_z1][i_pl].z2() >= filter_min){
                        temp_cluster.push_back(
                            temp_cluster_z1[i_clst_z1][i_pl]);
                    }
                }
                temp_cluster_z2.push_back(temp_cluster);
            }
        }
        /* cluster horizontally again */
        auto n_cluster_z2 = temp_cluster_z2.size();
        std::cout << "temp_cluster_z2 size: " << n_cluster_z2 << std::endl;
        for(auto i_clst_z2=0; i_clst_z2<n_cluster_z2; i_clst_z2++){
            auto n_pillar_z2 = temp_cluster_z2[i_clst_z2].size();
            for(auto i_pl=0; i_pl<n_pillar_z2; i_pl++){
                final_cluster.BringIn(temp_cluster_z2[i_clst_z2][i_pl], 1.0);
            }
        }
    }
    pillar_cluster_queue_.push(final_cluster);
}

void PillarClusterComponent::FormPlane(){
    
}

void PillarClusterComponent::FilterPlane(){
    
}

/* for rpclib server */
std::vector<std::vector<std::vector<double>>>
PillarClusterComponent::GetPillarClusterHorizon(){
    std::vector<std::vector<std::vector<double>>> result;
    std::vector<std::vector<double>> cluster;
    cluster.reserve(100);
    if(!pillar_cluster_horizon_queue_.empty()){
        auto &pillar_cluster = pillar_cluster_horizon_queue_.front();
        auto n_cluster = pillar_cluster.size();
        for(auto i=0; i<n_cluster; i++){
            cluster.clear();
            auto n_pillar = pillar_cluster[i].size();
            for(auto j=0; j<n_pillar; j++){
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

std::vector<std::vector<std::vector<double>>>
PillarClusterComponent::GetPillarCluster(){
    std::vector<std::vector<std::vector<double>>> result;
    std::vector<std::vector<double>> cluster;
    cluster.reserve(100);
    if(!pillar_cluster_queue_.empty()){
        auto &pillar_cluster = pillar_cluster_queue_.front();
        auto n_cluster = pillar_cluster.size();
        for(auto i=0; i<n_cluster; i++){
            cluster.clear();
            auto n_pillar = pillar_cluster[i].size();
            for(auto j=0; j<n_pillar; j++){
                cluster.push_back(pillar_cluster[i][j].GetCoor());
            }
            result.push_back(cluster);
        }
        return result;
    }else{
        std::cout << "pillar_cluster_queue_ is empty\n";
        return std::vector<std::vector<std::vector<double>>>();
    }
}