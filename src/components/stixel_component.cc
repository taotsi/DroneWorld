#include "components\stixel_component.h"
#include "kde.h"
#include <algorithm>
#include <cmath>

StixelComponent::StixelComponent
	(std::queue<ImageResponse>* disparity_retreived)
	:disparity_retreived_(disparity_retreived) {
	fov_ = PI / 4;
}

StixelComponent::~StixelComponent() {

}

void StixelComponent::Begin() {
	RunStixel();
    //auto pillar_frame = pillar_frame_queue_.front();
    //std::cout << pillar_frame.size() << std::endl;
    //pillar_frame[45][0].Print();
}

void StixelComponent::Update(double DeltaTime) {

}

void StixelComponent::RunStixel() {
	if (!disparity_retreived_->empty()) {
        // PUSH scaled_disparity_frame_queue_
		RetreiveStixel();
        disparity_retreived_->pop();				
	}
    if(!scaled_disparity_frame_queue_.empty()){
        // PUSH kde_frame_queue_
        Kde();
    }
	if(!kde_frame_queue_.empty()) {
		// PUSH kde_peak_frame_queue_
		FindKdePeak(0.5);
        kde_frame_queue_.pop();
	}
    if(!scaled_disparity_frame_queue_.empty()
        && !kde_peak_frame_queue_.empty()){
        // PUSH pillar_frame_queue_
        DetectObject();
        scaled_disparity_frame_queue_.pop();
        kde_peak_frame_queue_.pop();
    }
    std::cout << "ready\n";
}

void StixelComponent::RetreiveStixel(){
    // TODO: if roll != 0, correction needs to be done
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
    for (int i = i_start; i < width_; i += stixel_width_) {
        std::vector<double> stixel;
        for (int j = 0; j < height_; j++) {
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
	for (int i = 0; i < frame_scaled.size(); i++) {
        std::vector<double> kde_col;
        kde::RetreiveKde(frame_scaled[i], kde_col, 
            disp_max_, disp_min_, 1000);
		kde_frame.push_back(kde_col);
	}
	kde_frame_queue_.push(kde_frame);
}

/* finds and saves the position of peaks meeting certain conditions */
void StixelComponent::FindKdePeak(float delta_y) {
	auto &kde_frame = kde_frame_queue_.front();
	std::vector<std::vector<KdePeak>> kde_peak_frame;
	for (int i = 0; i < kde_frame.size(); i++) {
        std::vector<KdePeak> kde_peak;
        // yl = y * k * dmax / b / kw
        // y is physical vertical length
        // yl is the y-coordinate value of kde; 
        // b is baseline; kw is kde_width; 
        // k is j here, or x-coordinate of kde;
        // dmax is disparity max, = disp_max * width_
        // 0.01 is an exp value
        double slop = disp_max_*width_/baseline_/kde_width_*0.01;
        // 0.2m for filter window height
        double window_h_weight = 0.2/kde_width_*disp_max_/baseline_*width_;
        kde::RetreiveKdePeak(kde_frame[i], kde_peak, 0.7, 
            slop, 0.0, window_h_weight);
		if (kde_peak.empty()) {
			kde_peak_frame.push_back(
				std::vector<KdePeak>());
		} else {
			kde_peak_frame.push_back(kde_peak);
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
	double focus = baseline_ / (2 * disp_normalized*tan(fov_ / 2));
	p_camera.y_ = focus * baseline_ / (disp_normalized*width_);
	p_camera.z_ = (z_pixel - height_ / 2) * p_camera.y_ / focus;
	int x_pixel = stixel_width_ * x_pixel_scaled + stixel_width_ / 2;
	p_camera.x_ = (x_pixel - width_ / 2) * p_camera.y_ / focus;
	return p_camera;
}

/* returns world coordinate position */
// use rad instead of degree
Point3D StixelComponent::CameraToWorldCoor(
	Point3D &pos_camera, Point3D &p_camera, EulerAngle &angle) {
	Point3D p_world;
	double cos_theta = cos(angle.roll_);
	double sin_theta = sin(angle.roll_);
	double cos_omega = cos(angle.yaw_);
	double sin_omega = sin(angle.yaw_);
	double cos_phi = cos(angle.pitch_);
	double sin_phi = sin(angle.pitch_);
	// rotation
	p_world.x_ =
		p_camera.x_ * (cos_theta * cos_omega
			- sin_theta * sin_phi * sin_omega)
		- p_camera.y_ * (sin_theta * sin_phi)
		- p_camera.z_ * (cos_theta * sin_omega
			+ sin_theta * sin_phi*cos_omega);
	p_world.y_ =
		p_camera.x_ * (sin_theta * cos_omega
			+ sin_omega * cos_theta*sin_phi)
		+ p_camera.y_ * (cos_theta * cos_phi)
		+ p_camera.z_ * (-sin_theta * sin_omega
			+ cos_theta * sin_phi * cos_omega);
	p_world.z_ =
		p_camera.x_ * (sin_omega * cos_phi)
		- p_camera.y_ * (sin_phi)
		+p_camera.z_ * (cos_phi * cos_omega);
	// translation
	p_world.x_ += pos_camera.x_;
	p_world.y_ += pos_camera.y_;
	p_world.z_ += pos_camera.z_;
	return p_world;
}

/* Sliding-block filter */
void StixelComponent::DetectObject() {
	auto &scaled_disparity_frame =
		scaled_disparity_frame_queue_.front();
	auto &kde_peak_frame = kde_peak_frame_queue_.front();
	auto n_stixel = static_cast<int>(
        scaled_disparity_frame.data_[0].size());
    PillarFrame pillar_frame;
    // for each stixel in one frame
    for(auto stx_i=0; stx_i<kde_peak_frame.size(); stx_i++){
        std::vector<Pillar> pillar_col;
        BlockedIndex index {n_stixel};
        int n_peak = static_cast<int>(kde_peak_frame[stx_i].size());
        if(n_peak == 0){
            std::cout << "oops, no peak at all\n";
        }
        // for each peak in one stixel
        for(int peak_i=n_peak-1; peak_i>=0; peak_i--){
            Pillar pillar_temp;
            auto n_idx = index.size()-1;
            std::vector<int> idx_of_object;
            // for each unblocked segment
            for(auto idx=0; idx<n_idx; idx++){
                auto start = index[idx].second;
                auto end = index[idx+1].first;
                auto window_height = 
                    kde_peak_frame[stx_i][peak_i].window_height_;
                auto step_size = window_height/2;
                if(window_height < end - start){
                    double mean = static_cast<double>(
                        kde_peak_frame[stx_i][peak_i].pos_)
                        /kde_width_*disp_max_;
                    double min = static_cast<double>(
                        kde_peak_frame[stx_i][peak_i].window_left_)
                        /kde_width_*disp_max_;
                    double max = static_cast<double>(
                        kde_peak_frame[stx_i][peak_i].window_right_)
                        /kde_width_*disp_max_;
                    auto prev_stat = Filter(scaled_disparity_frame[stx_i], 
                            start, (end-start) % step_size, mean, min, max);
                    if(prev_stat.flag_ == kCompliant){
                        idx_of_object.push_back(start);
                        idx_of_object.push_back(start + (end-start) % step_size);
                    }
                    start += (end-start) % step_size;
                    while(start < end){
                        auto stat = Filter(scaled_disparity_frame[stx_i],
                        start, step_size, mean, min, max);
                        if(stat.flag_ == kCompliant){
                            idx_of_object.push_back(start);
                            idx_of_object.push_back(start+step_size);
                        }/*else if(stat.flag_ == kNotCompliant){
                            idx_of_object.push_back(stat.value_);
                        }*/
                        prev_stat = stat;
                        start += step_size;
                    }
                    if(!idx_of_object.empty()){
                        auto y_start = std::min_element(
                            idx_of_object.begin(), idx_of_object.end());
                        auto y_end = std::max_element(
                            idx_of_object.begin(), idx_of_object.end());
                        auto p_camera1 = GetCameraCoor(mean, stx_i, *y_start);
                        auto p_world1 =
                            CameraToWorldCoor(scaled_disparity_frame.pos_camera_, 
                                p_camera1, scaled_disparity_frame.angle_camera_);
                        auto p_camera2 = GetCameraCoor(mean, stx_i, *y_end);
                        auto p_world2 =
                            CameraToWorldCoor(scaled_disparity_frame.pos_camera_, 
                                p_camera2, scaled_disparity_frame.angle_camera_);
                        pillar_temp.SetPoint(p_world1);
                        pillar_temp.SetZ2(p_world2.z_);
                        pillar_col.push_back(pillar_temp);
                    }
                }
            }
        }
        pillar_frame.Push(pillar_col);
    }
    pillar_frame_queue_.push(pillar_frame);
}

void StixelComponent::Behave() {
	is_busy_ = true;
	thread_handle_ = std::thread{ 
		&StixelComponent::RunStixel, this };
}

// TODO: needs to be perfected
FilterStatus StixelComponent::Filter(
    std::vector<double> vec, int start, int step, 
    double mean, double min, double max){
    double sum = 0.0;
    double average = 0.0;
    double count = 0.0;
    for(int i=start; i<start+step; i++){
        sum += vec[i];
        count += 1.0;
    }
    average = sum/count;
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

/* for rpclib server */
std::vector<double> StixelComponent::GetKde() {
	if (!kde_frame_queue_.empty()) {
		// never ever pop it here
		return kde_frame_queue_.front()[45];
	} else {
		return std::vector<double>(width_/stixel_width_, 0.0);
	}
}
std::vector<std::vector<double>> StixelComponent::GetPillarFrame(){
    std::vector<std::vector<double>> pillars;
    if(!pillar_frame_queue_.empty()){
        auto &pillar_frame = pillar_frame_queue_.front();
        for(auto i=0; i<pillar_frame.size(); i++){
            for(auto j=0; j<pillar_frame[i].size(); j++){
                auto temp = pillar_frame[i][j].GetCoor();
                pillars.push_back(temp);
            }
        }
        return pillars;
    }else{
        std::cout << "pillar_frame_queue_ is empty\n";
        return std::vector<std::vector<double>>();
    }
}
