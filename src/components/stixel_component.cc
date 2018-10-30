#include <algorithm>
#include <cmath>

#include "components/stixel_component.h"
#include "math_utility.h"
#include "nlohmann/json.hpp"
#include "common_utility.h"

namespace droneworld{

using json = nlohmann::json;

StixelComponent::StixelComponent(
    std::queue<ImageResponse>* disparity_retreived)
	:disparity_retreived_(disparity_retreived) {
	SettingsJsonHandler settings;
    json &json_data = settings.GetJsonData()["CameraDefaults"]["CaptureSettings"];
    for(auto &it: json_data){
        if(it["ImageType"] == 4){
            if(!it["Width"].empty()){
                width_ = it["Width"];
            }
            if(!it["Height"].empty()){
                height_ = it["Height"];
            }
            if(!it["FOV_Degrees"].empty()){
                auto fov_dgr = it["FOV_Degrees"];
                fov_ = PI * static_cast<double>(fov_dgr) / 180.0;
            }
            break;
        }
    }
}

StixelComponent::~StixelComponent() {

}

void StixelComponent::Begin() {
	RunStixel();
}

void StixelComponent::Update(double DeltaTime) {

}

void StixelComponent::RunStixel() {
	if (!disparity_retreived_->empty()) {
        // PUSH scaled_disparity_frame_queue_
		RetreiveStixel();
        //auto &sf = scaled_disparity_frame_queue_.front();
        //disparity_retreived_->pop();	
        // std::cout << "stixel retreivation ready\n";		
	}
    if(!scaled_disparity_frame_queue_.empty()){
        // PUSH kde_frame_queue_
        Kde();
        // std::cout << "stixel kde ready\n";
    }
	if(!kde_frame_queue_.empty()) {
		// PUSH kde_peak_frame_queue_
		FindKdePeak(0.5);
        //kde_frame_queue_.pop();
        // std::cout << "stixel kde peak ready\n";
	}
    if(!scaled_disparity_frame_queue_.empty()
        && !kde_peak_frame_queue_.empty()){
        // PUSH pillar_frame_queue_
        DetectObject();
        //scaled_disparity_frame_queue_.pop();
        //kde_peak_frame_queue_.pop();
        // std::cout << "stixel detection ready\n";
    }
    if(!pillar_frame_queue_.empty()){
        //auto &pf = pillar_frame_queue_.front();
    }else{
        std::cout << "pillar_frame_queue_ is empty\n";
    }
    std::cout << "stixel reday\n";
}
void StixelComponent::RetreiveStixel(){
    auto &frame_raw = disparity_retreived_->front();
    Point3D pos_camera = TransformAirsimCoor(
        frame_raw.camera_position.x(),
        frame_raw.camera_position.y(),
        frame_raw.camera_position.z()
    );
    double quat_w = frame_raw.camera_orientation.w();
    double quat_x = frame_raw.camera_orientation.x();
    double quat_y = frame_raw.camera_orientation.y();
    double quat_z = frame_raw.camera_orientation.z();
    Quaternion angle_camera{ quat_w, quat_x, quat_y, quat_z };
    ScaledDisparityFrame frame_scaled{ pos_camera, angle_camera };
    int i_start = stixel_width_ / 2;
    std::vector<double> stixel;
    stixel.reserve(height_);
    for (int i = i_start; i < width_; i += stixel_width_) {
        stixel.clear();
        for (int j = height_-1; j >= 0; j--) {
            double pix_val = frame_raw.
                image_data_float[j*width_ + i];
            stixel.push_back(pix_val);
        }
        frame_scaled.PushStixel(stixel);
    }
    scaled_disparity_frame_queue_.push(frame_scaled);
}
/* gets kde and saves them */
void StixelComponent::Kde() {
    auto &frame_scaled = scaled_disparity_frame_queue_.front();
	std::vector<std::vector<double>> kde_frame;
    std::vector<double> kde_col;
    auto frame_size = frame_scaled.size();
	for (int i = 0; i < frame_size; i++) {
        kde_col.clear();
        RetreiveKde(frame_scaled[i], kde_col, 
            disp_max_, disp_min_, kde_width_);
		kde_frame.push_back(kde_col);
	}
	kde_frame_queue_.push(kde_frame);
}
/* finds and saves the position of peaks meeting certain conditions */
void StixelComponent::FindKdePeak(float delta_y) {
	auto &kde_frame = kde_frame_queue_.front();
	std::vector<std::vector<KdePeak>> kde_peak_frame;
    std::vector<KdePeak> kde_peaks;
	for (int i = 0; i < kde_frame.size(); i++) {
        kde_peaks.clear();;
        // yl = y * k * dmax / b / kw
        // y is physical vertical length
        // yl is the y-coordinate value of kde; 
        // b is baseline; kw is kde_width; 
        // k is j here, or x-coordinate of kde;
        // dmax is disparity max, = disp_max * width_
        // 0.121 is the value of the middle element of gauss kernel
        double slop = disp_max_*width_/baseline_/kde_width_*0.121;
        // 0.2m for filter window height
        double window_h_weight = 0.2/kde_width_*disp_max_/baseline_*width_;
        RetreiveKdePeak(kde_frame[i], kde_peaks, disp_max_, disp_min_, 
            0.8, slop, 5.5, window_h_weight);
		if (kde_peaks.empty()) {
			kde_peak_frame.push_back(
				std::vector<KdePeak>());
		} else {
			kde_peak_frame.push_back(kde_peaks);
		}
	}
	kde_peak_frame_queue_.push(kde_peak_frame);
}
/*
our coordinate system:
z
|  y
|/
0----x
yet in Airsim, +X is North, +Y is East and +Z is Down(NED system)
ours is also different with dual-camera's coordinate
*/
Point3D StixelComponent::TransformAirsimCoor(
	double x, double y, double z) {
	return Point3D{ y, x, -z };
}
Point3D StixelComponent::GetCameraCoor(
	double disp_normalized, int x_pixel_scaled, int z_pixel) {
	Point3D p_camera;
	double focus = width_/2/tan(fov_/2);
	p_camera.y_ = (baseline_/2/tan(fov_/2)) / disp_normalized;
	p_camera.z_ = (z_pixel - height_/2) * p_camera.y_ / focus;
	int x_pixel = stixel_width_ * x_pixel_scaled + stixel_width_/2;
	p_camera.x_ = (x_pixel - width_/2) * p_camera.y_ / focus;
	return p_camera;
}
/* returns world coordinate position */
// use rad instead of degree
Point3D StixelComponent::CameraToWorldCoor(
	Point3D &pos_camera, Point3D &p_camera, EulerAngle &angle) {
	Point3D p_world;
	double cos_omega = cos(angle.yaw_);
	double sin_omega = sin(angle.yaw_);
	// rotation
    // because of gimbal, only yaw changes
    p_world.x_ = cos_omega*p_camera.x_ + sin_omega*p_camera.y_;
    p_world.y_ = cos_omega*p_camera.y_ - sin_omega*p_camera.x_; 
    p_world.z_ = p_camera.z_;
	// translation
	p_world.x_ += pos_camera.x_;
	p_world.y_ += pos_camera.y_;
	p_world.z_ += pos_camera.z_;
	return p_world;
}

/* Sliding-block filter */
void StixelComponent::DetectObject() {
	auto &scaled_disparity_frame = scaled_disparity_frame_queue_.front();
	auto &kde_peak_frame = kde_peak_frame_queue_.front();
    std::vector<Pillar> pillar_frame;
    auto n_stixel = kde_peak_frame.size();
    // for each stixel in one frame
    for(auto stx_i=0; stx_i<n_stixel; stx_i++){
        BlockedIndex index {height_};
        int n_peak = static_cast<int>(kde_peak_frame[stx_i].size());
        // for each peak in one stixel
        for(int peak_i=n_peak-1; peak_i>=0; peak_i--){
            Pillar pillar_temp;
            auto n_idx = index.size()-1;
            std::vector<int> idx_of_object;
            // for each unblocked segment
            for(auto idx=0; idx<n_idx; idx++){
                auto start = index[idx].second;
                auto end = index[idx+1].first;
                int window_height = kde_peak_frame[stx_i][peak_i].window_height_;
                int step_size = window_height>>1;
                if(window_height < end - start){
                    int temp_step = (end-start) % step_size;
                    if(temp_step != 0){
                        auto prev_stat = Filter(scaled_disparity_frame[stx_i], 
                            start, temp_step, 
                            kde_peak_frame[stx_i][peak_i].mean_, 
                            kde_peak_frame[stx_i][peak_i].left_, 
                            kde_peak_frame[stx_i][peak_i].right_);
                        if(prev_stat.flag_ == kCompliant){
                            idx_of_object.push_back(start);
                            idx_of_object.push_back(start + temp_step);
                        }
                        start += temp_step;
                    }
                    while(start < end){
                        auto stat = Filter(scaled_disparity_frame[stx_i], 
                                start, step_size, 
                                kde_peak_frame[stx_i][peak_i].mean_, 
                                kde_peak_frame[stx_i][peak_i].left_, 
                                kde_peak_frame[stx_i][peak_i].right_);
                        if(stat.flag_ == kCompliant){
                            idx_of_object.push_back(start);
                            idx_of_object.push_back(start+step_size);
                        }
                        start += step_size;
                    }
                    if(!idx_of_object.empty()){
                        std::vector<std::pair<int, int>> object_z1z2;
                        LayerObject(idx_of_object, object_z1z2, window_height);
                        for(auto &it : object_z1z2){
                            index.AddSegment(it.first, it.second);
                        }
                        for(auto &it : object_z1z2){
                            Line2dFitted line{static_cast<double>(it.first), 
                                scaled_disparity_frame[stx_i][it.first], 
                                static_cast<double>(it.second), 
                                scaled_disparity_frame[stx_i][it.second]};
                            if(it.second - it.first > 1){
                                for(auto i=it.first; i<it.second; i++){
                                    line.AddPoint(static_cast<double>(i),
                                        scaled_disparity_frame[stx_i][i]);
                                }
                            }
                            // if it's ground, the slope is a fixed value.
                            // here it's about 7.1e-5
                            if(abs(line.GetSlope()) < 6.0e-5){
                                auto p_camera1 = GetCameraCoor(
                                    kde_peak_frame[stx_i][peak_i].mean_, 
                                    stx_i, it.first);
                                auto p_camera2 = GetCameraCoor(
                                    kde_peak_frame[stx_i][peak_i].mean_, 
                                    stx_i, it.second);
                                auto p_world1 = CameraToWorldCoor(
                                    scaled_disparity_frame.pos_camera_, 
                                    p_camera1, 
                                    scaled_disparity_frame.angle_camera_);
                                auto p_world2 = CameraToWorldCoor(
                                    scaled_disparity_frame.pos_camera_, 
                                    p_camera2, 
                                    scaled_disparity_frame.angle_camera_);
                                pillar_temp.SetPoint(p_world1);
                                pillar_temp.SetZ2(p_world2.z_);
                                pillar_frame.push_back(pillar_temp);
                            }else{
                                // std::cout << "DetectObject(), this might be ground or a roof\n";
                            }
                        }
                    }
                }
            }
        }
    }
    pillar_frame_queue_.push(pillar_frame);
}

void StixelComponent::LayerObject(std::vector<int> &object_idx,
    std::vector<std::pair<int, int>> &results, int h_thh) {
    results.clear();
    if(!object_idx.empty()){
        if(!std::is_sorted(object_idx.begin(), object_idx.end())){
            std::sort(object_idx.begin(), object_idx.end());
        };
        std::pair<int, int> temp{object_idx[0], object_idx[0]};
        for(auto i=0; i<object_idx.size()-1; i++){
            if(object_idx[i+1]-object_idx[i] > h_thh){
                temp.second = object_idx[i];
                results.push_back(temp);
                temp.first = object_idx[i+1];
            }
        }
        temp.second = *(object_idx.end()-1);
        results.push_back(temp);
    }else{
        std::cout << "layering object: no object info!\n";
    }
}

void StixelComponent::PrintDisp(int col){
    if(!scaled_disparity_frame_queue_.empty()){
        auto disp = scaled_disparity_frame_queue_.front();
        int n_col = static_cast<int>(disp.size());
        int count = 0;
        if(!(col<0 || col >=n_col)){
            for(auto &it : disp.data_[col]){
                if(it < disp_max_ && it > disp_min_){
                    std::cout << std::setprecision(3) << it << "  ";
                    count++;
                }
            }
            std::cout << "( " << count << " )\n";
        }else{
            std::cout << "PrintDisp out of range\n";
        }
    }else{
        std::cout << "scaled_disparity_frame_queue_ is empty\n";
    }
}
void StixelComponent::PrintKde(int col){
    if(!kde_frame_queue_.empty()){
        auto kde = kde_frame_queue_.front();
        int n_col = static_cast<int>(kde.size());
        if(!(col<0 || col>=n_col)){
            for(auto &v : kde[col]){
                std::cout << std::setprecision(3) << v << "  ";
            }
            std::cout << "\n";
        }else{
            std::cout << "PrintKde out of range\n";
        }
    }else{
        std::cout << "kde_frame_queue_ is empty\n";
    }
}

/* for rpclib server */
std::vector<std::vector<double>> StixelComponent::GetDisparityFrame(){
    if(!scaled_disparity_frame_queue_.empty()){
        return scaled_disparity_frame_queue_.front().data_;
    }else{
        std::cout << "scaled_disparity_frame_queue_ is empty\n";
        return std::vector<std::vector<double>>();
    }
}
std::vector<std::vector<double>> StixelComponent::GetKde() {
	if (!kde_frame_queue_.empty()) {
		// never ever pop it here
		return kde_frame_queue_.front();
	} else {
        std::cout << "kde_frame_queue_ is empty\n";
		return std::vector<std::vector<double>>();
	}
}
std::vector<std::vector<double>> StixelComponent::GetPillarFrame(){
    std::vector<std::vector<double>> pillars;
    pillars.reserve(150);
    if(!pillar_frame_queue_.empty()){
        auto &pillar_frame = pillar_frame_queue_.front();
        for(auto i=0; i<pillar_frame.size(); i++){
                auto temp = pillar_frame[i].GetCoor();
                pillars.push_back(temp);
        }
        return pillars;
    }else{
        std::cout << "pillar_frame_queue_ is empty\n";
        return std::vector<std::vector<double>>();
    }
}
}