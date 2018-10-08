#pragma once
#include <vector>
#include <algorithm>
#include "data_type.h"

namespace kde{

static std::vector<double> KERNEL_GAUSS_17 = {
    0.00678462, 0.0133395, 0.0239336, 0.0392686, 0.0588726,		
    0.0806656, 0.101032, 0.115629, 0.12095, 0.115629, 
    0.101032, 0.0806656, 0.0588726, 0.0392686, 0.0239336, 
    0.0133395, 0.00678462};

void RetreiveKde(std::vector<double> const &src, 
    std::vector<double> &dst, double max, double min, int kde_width, 
    std::vector<double> const &kernel=KERNEL_GAUSS_17) {
    if(!dst.empty()){
        dst.clear();
    }
    dst.reserve(kde_width);
    dst.resize(kde_width);
    std::fill(dst.begin(), dst.end(), 0.0);
    auto kernel_size = kernel.size();
    auto kernel_half_size = kernel.size()/2;
    for(auto i=0; i<src.size(); i++){
        if(src[i] <= max && src[i] >= min){
            int kde_x = static_cast<int>(src[i] * kde_width / max);
            for(auto j=0; j<kernel_size; j++){
                if(kde_x-kernel_half_size >= 0 &&
                    kde_x+kernel_half_size <= kde_width-1){
                        dst[kde_x-kernel_half_size+j] += kernel[j];
                }
            }
        }
    }
}

void RetreiveKdePeak(std::vector<double> const &kde, 
    std::vector<KdePeak> &peaks, double threashhold_weight, 
    double kde_slope, double kde_y_intercept, 
    double window_height_weight){
    if(!peaks.empty()){
        peaks.clear();
    }
    // std::cout << "------ retreive peak ------\n";
    // 1 for ascending and -1 for descending
    int prev_dir = kde[1] > kde[0] ? 1 : -1;
    auto kde_width = kde.size();
    for(auto i=1; i<kde_width-1; i++){
        int crt_dir = kde[i+1] > kde[i] ? 1 : -1;
        if(prev_dir == 1 && crt_dir == -1){
            //std::cout << "found a peak\n";
            if(kde[i] > kde_slope*i + kde_y_intercept){
                // std::cout << "peak's good\n";
                KdePeak peak{i};
                auto thh = kde[i] * threashhold_weight;
                int il = i, ir = i;
                while(kde[il] > thh) { il--; }
                while(kde[ir] > thh) { ir++; }
                int window_height = static_cast<int>(
                    i * window_height_weight);
                peak.SetWindow(il, ir, window_height);
                peaks.push_back(peak);
            }
        }
        prev_dir = crt_dir;
    }
}

} // namespace kde