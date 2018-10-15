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
        Cluster();
        //pillar_frame_queue_->pop();
        std::cout << "cluster ready\n";
    }
}

void PillarClusterComponent::Cluster(){
    auto &pillar_frame = pillar_frame_queue_->front();
    PillarClusters pillar_cluster_horizon;
    for(auto i=0; i<pillar_frame.size(); i++){
        pillar_cluster_horizon.BringIn(pillar_frame[i], 1.0, 0.3); // 1m and 0.3m
    }
    pillar_cluster_queue_.push(pillar_cluster_horizon);
}

ComplementStatus PillarClusterComponent::CompletePillar(Pillar &pillar, 
    double z_max, double z_min, double h_thh, bool is_forcibly=false){
    bool is_head = false, is_sill = false, is_jamb = false;
    if(z_max-pillar.z2() < h_thh && pillar.z1()-z_min < h_thh){// jamb
        pillar.SetZ1(z_min);
        pillar.SetZ2(z_max);
        return kJamb;
    }else if(z_max-pillar.z2() < h_thh && pillar.z1()-z_min > h_thh){// head
        pillar.SetZ2(z_max);
        return kHead;
    }else if(z_max-pillar.z2() > h_thh && pillar.z1()-z_min < h_thh){// sill
        pillar.SetZ1(z_min);
        return kSill;
    }else{// grille, taken as jamb, temporarily
        pillar.SetZ1(z_min);
        pillar.SetZ2(z_max);
        return kJamb;
    }
}

void PillarClusterComponent::FormPlane(){
    
}

void PillarClusterComponent::FilterPlane(){
    
}

/* for rpclib server */
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

/* backup code */
// TODO: this methods fails, deprecate or perfect it
/*
void PillarClusterComponent::VerticalCluster(){
    auto &cluster_horizon = pillar_cluster_horizon_queue_.front();
    auto n_cluster_horizon = cluster_horizon.size();
    PillarClusters final_cluster;
    std::vector<std::vector<Pillar>> temp_cluster_z1, temp_cluster_z2;
    std::vector<double> kde_z1, kde_z2;
    std::vector<KdePeak> z1_peaks, z2_peaks;
    for(auto i_clst_h=0; i_clst_h<n_cluster_horizon; i_clst_h++){
        std::cout << "------ original cluster " << i_clst_h << " ------\n";
        kde_z1.clear();                kde_z2.clear();
        z1_peaks.clear();              z2_peaks.clear();
        temp_cluster_z1.clear();      temp_cluster_z2.clear();
        auto &z1_vec = cluster_horizon[i_clst_h].z1_vec_;
        auto &z2_vec = cluster_horizon[i_clst_h].z2_vec_;
        auto max_z1 = std::max_element(z1_vec.begin(), z1_vec.end());
        auto min_z1 = std::min_element(z1_vec.begin(), z1_vec.end());
        auto max_z2 = std::max_element(z2_vec.begin(), z2_vec.end());
        auto min_z2 = std::min_element(z2_vec.begin(), z2_vec.end());
        //int z1_vec_size = static_cast<int>(z1_vec.size());
        //int z2_vec_size = static_cast<int>(z2_vec.size());
        kde::RetreiveKde(z1_vec, kde_z1, *max_z1, *min_z1, 30);
        kde::RetreiveKdePeak(kde_z1, z1_peaks, *max_z1, *min_z1, 
            0.3, 0, 0.25, 0.2);
        kde::RetreiveKde(z2_vec, kde_z2, *max_z2, *min_z2, 30);
        kde::RetreiveKdePeak(kde_z2, z2_peaks, *max_z2, *min_z2, 
            0.3, 0, 0.25, 0.2);
        std::cout << "---- kde_z1, size: " << kde_z1.size() << "\n";
        for(auto itr : kde_z1){
            std::cout << itr << "  ";
        }std::cout << "\n";
        std::cout << "-- z1 min, max =  " << *min_z1 << ", " << *max_z1 << "\n";
        std::cout << "---- kde_z2, size: " << kde_z2.size() << "\n";
        for(auto itr : kde_z2){
            std::cout << itr << "  ";
        }std::cout << "\n";
        std::cout << "-- z2 min, max =  " << *min_z2 << ", " << *max_z2 << "\n";
        
        std::vector<Pillar> temp_cluster;
        auto n_peaks_z1 = z1_peaks.size();
        auto n_pillar = cluster_horizon[i_clst_h].size();
        std::cout << "horizon cluster has pillar: " << n_pillar << std::endl;
        std::cout << "z1 peaks size: " << n_peaks_z1 << "\n";
        int count = 0;
        for(auto i_pk_z1=0; i_pk_z1<n_peaks_z1; i_pk_z1++){
            temp_cluster.clear();
            z1_peaks[i_pk_z1].Print();
            for(auto i_pl=0; i_pl<n_pillar; i_pl++){
                if(cluster_horizon[i_clst_h][i_pl].z1() <= z1_peaks[i_pk_z1].right_
                    && cluster_horizon[i_clst_h][i_pl].z1() >= z1_peaks[i_pk_z1].left_){
                    temp_cluster.push_back(cluster_horizon[i_clst_h][i_pl]);
                    count++;
                }else{
                    std::cout << "\tz1, pillar not good\n";
                }
            }
            std::cout << "\tz1 peak " << i_pk_z1 << ", z1 takes in " << count << " pillars\n";
            if(!temp_cluster.empty()){
                temp_cluster_z1.push_back(temp_cluster);
                std::cout << "z1 has " << temp_cluster_z1.size() << " clusters now\n";
            }else{
                
            }
        }
        
        auto n_peaks_z2 = z2_peaks.size();
        auto n_cluster_z1 = temp_cluster_z1.size();
        std::cout << "z1 cluster size: " << n_cluster_z1 << std::endl;
        std::cout << "z2_peaks size: " << n_peaks_z2 << std::endl;
        count = 0;
        for(auto i_pk_z2=0; i_pk_z2<n_peaks_z2; i_pk_z2++){
            temp_cluster.clear();
            z2_peaks[i_pk_z2].Print();
            for(auto i_clst_z1=0; i_clst_z1<n_cluster_z1; i_clst_z1++){
                auto n_pillar_per_cluster_z1 = temp_cluster_z1[i_clst_z1].size();
                for(auto i_pl=0; i_pl<n_pillar_per_cluster_z1; i_pl++){
                    if(temp_cluster_z1[i_clst_z1][i_pl].z2() <= z2_peaks[i_pk_z2].right_
                        && temp_cluster_z1[i_clst_z1][i_pl].z2() >= z2_peaks[i_pk_z2].left_){
                        temp_cluster.push_back(
                            temp_cluster_z1[i_clst_z1][i_pl]);
                        count++;
                    }else{
                        std::cout << "\tz2, pillar not good\n";
                    }
                }
                std::cout << "\tz2 peak " << i_pk_z2 << ", z2 takes in " << count << " pillars\n";
                if(!temp_cluster.empty()){
                    temp_cluster_z2.push_back(temp_cluster);
                    std::cout << "z2 has " << temp_cluster_z2.size() << " clusters now\n";
                }else{
                    
                }
            }
        }
        
        auto n_cluster_z2 = temp_cluster_z2.size();
        std::cout << "z2 cluster size: " << n_cluster_z2 << std::endl;
        for(auto i_clst_z2=0; i_clst_z2<n_cluster_z2; i_clst_z2++){
            auto n_pillar_per_cluster_z2 = temp_cluster_z2[i_clst_z2].size();
            for(auto i_pl=0; i_pl<n_pillar_per_cluster_z2; i_pl++){
                // 1.0 for 1m
                final_cluster.BringIn(temp_cluster_z2[i_clst_z2][i_pl], 1.0);
            }
        }
    }
    pillar_cluster_queue_.push(final_cluster);
}
*/