#include "components/pillar_cluster_component.h"
#include "math_utility.h"

namespace droneworld{

extern FilterStatus Filter(
    std::vector<double> &vec, int start, int step, 
    double mean, double min, double max);

PillarClusterComponent::PillarClusterComponent(
    std::queue<std::vector<Pillar>>* pillar_frame_queue)
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
        // PUSH primary_pillar_cluster_queue_
        PrimaryCluster();
        //pillar_frame_queue_->pop();
        std::cout << "cluster primary ready\n";
    }
    if(!primary_pillar_cluster_queue_.empty()){
        // PUSH filtered_cluster_queue_
        ComplementFilter();
        //primary_pillar_cluster_queue_.pop()
        auto &cluster = primary_pillar_cluster_queue_.front();
        std::cout << "--- primary cluster size: " << cluster.size() << "\n";
        std::cout << "cluster complement ready\n";
    }else{
        std::cout << "primary_pillar_cluster_queue_ is empty\n";
    }
    if(!filtered_cluster_queue_.empty()){
        auto &cluster = filtered_cluster_queue_.front();
        std::cout << "--- filtered cluster size: " << cluster.size() << "\n";
    }
    std::cout << "\n\n";
}

void PillarClusterComponent::PrimaryCluster(){
    auto &pillar_frame = pillar_frame_queue_->front();
    PillarClustersHorizon pillar_cluster_horizon;
    for(auto i=0; i<pillar_frame.size(); i++){
        pillar_cluster_horizon.BringIn(pillar_frame[i], 1.0, 0.3); // 1m and 0.3m
    }
    primary_pillar_cluster_queue_.push(pillar_cluster_horizon);
}

void PillarClusterComponent::ComplementFilter(){
    auto &pillar_cluster = primary_pillar_cluster_queue_.front();
    std::vector<std::vector<Pillar>> filtered_clusters;
    auto n_pillar_cluster = pillar_cluster.size();
    for(auto i=0; i<n_pillar_cluster; i++){
        std::cout << "---------------- primary cluster " << i << "\n";
        ComplementCluster(pillar_cluster[i], filtered_clusters);
    }
    filtered_cluster_queue_.push(filtered_clusters);
}

void PillarClusterComponent::ComplementCluster(
    SinglePillarCluster &clst_src, std::vector<std::vector<Pillar>> &clst_dst) {
    double drone_height = 0.3;
    double drone_width = 1.0;
    double z_max = clst_src.z_max();
    double z_min = clst_src.z_min();
    std::vector<Pillar> jambs, heads, sills;
    int idx = 0;
    int n_clst_src = clst_src.size();
    int window_start = 0, window_end = 0;
    while(idx < n_clst_src){
        if(idx >= n_clst_src){
            std::cout << "\tthis primary cluster over\n";
        }
        jambs.clear();
        heads.clear();
        sills.clear();
        double z_max_window = z_max;
        double z_min_window = z_min;
        std::cout << "\tstart finding jambs on the left\n";
        // find jambs on the left
        while(true){
            if(idx >= n_clst_src){
                std::cout << "\t** find jambs left over\n";
                if(!jambs.empty()){
                    clst_dst.push_back(jambs);
                    jambs.clear();
                }
                break;
            }
            auto stat_temp = CompletePillar(
                clst_src[idx], z_max, z_min, drone_height);
            if(stat_temp == kJamb){
                std::cout << "\t\tjamb, jambs left " << idx << "\n";
                jambs.push_back(clst_src[idx]);
                idx++;
            }else if(stat_temp == kSill){
                std::cout << "\t\t** found a sill, jambs left over\n";
                sills.push_back(clst_src[idx]);
                z_min_window = clst_src[idx].z2();
                idx++;
                window_start = idx;
                break;
            }else{ // kHead
                std::cout << "\t\t** found a head, jambs left over\n";
                heads.push_back(clst_src[idx]);
                z_max_window = clst_src[idx].z1();
                idx++;
                window_start = idx;
                break;
            }
        }
        std::cout << "\tstart finding sills and heads\n";
        // find sills and heads
        while(true){
            if(idx >= n_clst_src){
                std::cout << "\t** find sills & heads over\n";
                // TODO: if not empty
                if(!jambs.empty()){
                    std::cout << "\tpush jambs\n";
                    clst_dst.push_back(jambs);
                    jambs.clear();
                }
                if(!sills.empty()){
                    clst_dst.push_back(sills);
                    sills.clear();
                }
                if(!heads.empty()){
                    clst_dst.push_back(heads);
                    heads.clear();
                }
                break;
            }
            auto stat_temp = CompletePillar(
                clst_src[idx], z_max, z_min, drone_height);
            if(stat_temp == kJamb){
                std::cout << "found a jamb, sills & heads over\n";
                window_end = idx;
                int start_temp = window_start>1 ? window_start-1 : 0;
                double xl = clst_src[start_temp].x();
                double yl = clst_src[start_temp].y();
                double xr = clst_src[window_end].x();
                double yr = clst_src[window_end].y();
                if(pow(xl-xr, 2) + pow(yl-yr, 2) > pow(drone_width, 2)){
                    std::cout << "window wide enough\n";
                    if(!jambs.empty()){
                        clst_dst.push_back(jambs);
                        jambs.clear();
                    }
                    if(!sills.empty()){
                        clst_dst.push_back(sills);
                        sills.clear();
                    }
                    if(!heads.empty()){
                        clst_dst.push_back(heads);
                        heads.clear();
                    }
                }else{
                    std::cout << "window too narrow, fills them\n";
                    FillWindow(jambs, heads, sills, z_max, z_min);
                    clst_dst.push_back(jambs);
                    jambs.clear();
                }
                window_start = idx;
                idx++;
                break;
            }else{
                if(stat_temp == kSill){
                    std::cout << "found a sill, sills & heads goes on\n";
                    if(clst_src[idx].z2() > z_min_window){
                       z_min_window = clst_src[idx].z2();
                    }
                    sills.push_back(clst_src[idx]);
                }else{ // kHead
                    std::cout << "found a head, sills & heads goes on\n";
                    if(clst_src[idx].z1() < z_max_window){
                        z_max_window = clst_src[idx].z1();
                    }
                    heads.push_back(clst_src[idx]);
                }
                if(z_max_window - z_min_window < drone_height){
                    std::cout << "window too short on height, fills them";
                    FillWindow(jambs, heads, sills, z_max, z_min);
                    CompletePillar(clst_src[idx], z_max, z_min, 0, true);
                    window_start = idx;
                    idx++;
                }else{
                    idx++;
                }
            }
        }
    }
}

ComplementStatus PillarClusterComponent::CompletePillar(Pillar &pillar, 
    double z_max, double z_min, double h_thh, bool is_forcibly) {
    if(!is_forcibly){
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
    }else{
        pillar.SetZ2(z_max);
        pillar.SetZ1(z_min);
        return kJamb;
    }
}

void PillarClusterComponent::FillWindow(std::vector<Pillar> jambs, 
    std::vector<Pillar> heads, std::vector<Pillar> sills, 
    double z_max, double z_min){
    auto n_sills = sills.size();
    for(auto i=0; i<n_sills; i++){
        CompletePillar(sills[i], z_max, z_min, 0, true);
    }
    jambs.insert(jambs.end(), sills.begin(), sills.end());
    auto n_heads = heads.size();
    for(auto i=0; i<n_heads; i++){
        CompletePillar(heads[i], z_max, z_min, 0, true);
    }
    jambs.insert(jambs.end(), heads.begin(), heads.end());
}

/* for rpclib server */
std::vector<std::vector<std::vector<double>>>
PillarClusterComponent::GetPillarCluster(){
    std::vector<std::vector<std::vector<double>>> result;
    std::vector<std::vector<double>> cluster;
    cluster.reserve(30);
    if(!primary_pillar_cluster_queue_.empty()){
        auto &pillar_cluster = primary_pillar_cluster_queue_.front();
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
        std::cout << "primary_pillar_cluster_queue_ is empty\n";
        return std::vector<std::vector<std::vector<double>>>();
    }
}

std::vector<std::vector<std::vector<double>>> 
PillarClusterComponent::GetFilteredCluster(){
    std::vector<std::vector<std::vector<double>>> result;
    std::vector<std::vector<double>> cluster;
    cluster.reserve(30);
    if(!filtered_cluster_queue_.empty()){
        auto &pillar_cluster = filtered_cluster_queue_.front();
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
        std::cout << "filtered_cluster_queue_ is empty\n";
        return std::vector<std::vector<std::vector<double>>>();
    }
}

/* backup code */
// TODO: this methods fails, deprecate or perfect it
/*
void PillarClusterComponent::VerticalCluster(){
    auto &cluster_horizon = pillar_cluster_horizon_queue_.front();
    auto n_cluster_horizon = cluster_horizon.size();
    PillarClustersHorizon final_cluster;
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
        RetreiveKde(z1_vec, kde_z1, *max_z1, *min_z1, 30);
        RetreiveKdePeak(kde_z1, z1_peaks, *max_z1, *min_z1, 
            0.3, 0, 0.25, 0.2);
        RetreiveKde(z2_vec, kde_z2, *max_z2, *min_z2, 30);
        RetreiveKdePeak(kde_z2, z2_peaks, *max_z2, *min_z2, 
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
    primary_pillar_cluster_queue_.push(final_cluster);
}
*/
}