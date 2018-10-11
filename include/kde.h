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

inline void RetreiveKde(std::vector<double> const &src, 
    std::vector<double> &dst, double max, double min, int kde_width, 
    std::vector<double> const &kernel=KERNEL_GAUSS_17) {
    if(!dst.empty()){
        dst.clear();
    }
    dst.reserve(kde_width);
    dst.resize(kde_width);
    std::fill(dst.begin(), dst.end(), 0.0);
    int kernel_size = static_cast<int>(kernel.size());
    int kernel_half_size = kernel_size>>1;
    for(auto i=0; i<src.size(); i++){
        // std::cout << "retreiving kde, index = " << i << "\n";
        if(src[i] < max && src[i] > min){
            int kde_x = static_cast<int>(
                (src[i]-min)/(max-min)*(kde_width-1));
            //std::cout << "kde_x = " << kde_x << "\n";
            for(auto j=0; j<kernel_size; j++){
                if(kde_x-kernel_half_size >= 0 &&
                    kde_x+kernel_half_size <= kde_width-1){
                        //std::cout << kde_x-kernel_half_size+j << "\n";
                        dst[kde_x-kernel_half_size+j] += kernel[j];
                }
            }
        }
    }
}

inline void RetreiveKdePeak(std::vector<double> const &kde, 
    std::vector<KdePeak> &peaks, double max, double min, 
    double threashhold_weight, double kde_slope, 
    double kde_y_intercept, double window_height_weight){
    if(!peaks.empty()){
        peaks.clear();
    }
    if(threashhold_weight > 1){
        threashhold_weight = 1;
        std::cout << "threashhold_weight for RetreiveKdePeak is bigger than 1\n";
    }else if(threashhold_weight<0){
        threashhold_weight = 0;
        std::cout << "threashhold_weight for RetreiveKdePeak is less than 0\n";
    }
    // std::cout << "------ retreive peak ------\n";
    auto kde_width = kde.size();
    int offset = static_cast<int>(
        static_cast<double>(kde_width-1)/(max-min)*min);
    // 1 for ascending and -1 for descending
    int prev_dir = kde[1] > kde[0] ? 1 : -1;
    for(auto i=1; i<kde_width-1; i++){
        int crt_dir = kde[i+1] > kde[i] ? 1 : -1;
        if(prev_dir == 1 && crt_dir == -1){
            //std::cout << "found a peak\n";
            if(kde[i] > kde_slope*i + kde_y_intercept){
                //std::cout << "peak's good\n";
                KdePeak peak;
                auto thh = kde[i] * threashhold_weight;
                int il = i, ir = i;
                while(kde[il] > thh && il > 0) { 
                    if(kde[il] > kde[il-1]){
                        il--;
                    }else{
                        break;
                    }
                }
                while(kde[ir] > thh && ir < kde_width-1) {
                    if(kde[ir] > kde[ir+1]){
                        ir++;
                    }else{
                        break;
                    }
                }
                int window_height = static_cast<int>(
                    (i+offset) * window_height_weight);
                peak.SetWindowHeight(window_height);
                double mean = static_cast<double>(i)
                    /static_cast<double>(kde_width-1)*(max-min)+min;
                double left = static_cast<double>(il)
                    /static_cast<double>(kde_width-1)*(max-min)+min;
                double right = static_cast<double>(ir)
                    /static_cast<double>(kde_width-1)*(max-min)+min;
                peak.SetVal(mean, right, left);
                peaks.push_back(peak);
            }
        }
        prev_dir = crt_dir;
    }
}

inline FilterStatus Filter(
    std::vector<double> const &vec, int start, int step, 
    double mean, double min, double max){
    double sum = 0.0;
    double average = 0.0;
    double count = 0.0;
    if(step == 0){
        std::cout << "step = 0\n";
    }
    for(int i=start; i<start+step; i++){
        sum += vec[i];
        count += 1.0;
    }
    average = sum > 0.00001 ? sum/count : 0.0;
    // std::cout << sum << " " << count << " " << average << " " << mean << " " << min << " " << max << "\n";
    if(average <= max && average >= min){
        auto flag = kCompliant;
        FilterStatus stat{flag};
        return stat;
    }else{
        // temporary code
        int pos = start+step/2;
        auto flag = kNotCompliant;
        //std::cout << "not compliant\n";
        FilterStatus stat{flag, pos};
        return stat; 
    }
}

} // namespace kde