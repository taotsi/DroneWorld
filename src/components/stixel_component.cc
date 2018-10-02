#include "components\stixel_component.h"
#include <algorithm>
#include <cmath>

StixelComponent::StixelComponent
	(std::queue<ImageResponse>* disparity_retreived)
	:disparity_retreived_(disparity_retreived) {
	W17_GAUSS = {
		0.00678462 / width_,	0.0133395 / width_, 
		0.0239336 / width_,		0.0392686 / width_, 
		0.0588726 / width_,		0.0806656 / width_, 
		0.101032 / width_,		0.115629 / width_, 
		0.12095 / width_,		0.115629 / width_, 
		0.101032 / width_,		0.0806656 / width_, 
		0.0588726 / width_,		0.0392686 / width_, 
		0.0239336 / width_,		0.0133395 / width_, 
		0.00678462 / width_ };
	fov_ = PI / 4;
}

StixelComponent::~StixelComponent() {

}

void StixelComponent::Begin() {
	Stixel();
}

void StixelComponent::Update(double DeltaTime) {

}

void StixelComponent::Stixel() {
	if (!disparity_retreived_->empty()) {
        // PUSH scaled_disparity_frame_queue_
		// PUSH kde_frame_queue_
		Kde();
        disparity_retreived_->pop();				
	}
	if (!kde_frame_queue_.empty()) {
		// PUSH kde_peak_frame_queue_
		FindKdePeakPos(0.5);
        kde_frame_queue_.pop();
	}
	if (!scaled_disparity_frame_queue_.empty()
		&& !kde_peak_frame_queue_.empty()) {
		// PUSH pillar_frame_queue_
		DetectObject();
        scaled_disparity_frame_queue_.pop();
        kde_peak_frame_queue_.pop();
	}
}

/* gets kde and saves them */
void StixelComponent::Kde() {
	/* get a frame of disparity if available */
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
	/* kernel density estimate */
	std::vector<std::vector<double>> temp_frame;
	for (int i = 0; i < frame_scaled.data_.size(); i++) {
		std::vector<double> temp_stixel(kde_width_, 0);
		for (int j = 0; j < height_; j++) {
			if (frame_scaled.data_[i][j] <= disp_max_ &&
				frame_scaled.data_[i][j] >= disp_min_) {
				int kde_val = static_cast<int>(
					frame_scaled.data_[i][j] * kde_width_ / disp_max_);
				for (int k = 0; k < 17; k++) {
					if (kde_val - 8 >= 0 &&
						kde_val + 8 <= kde_width_ - 1) {
						// W17_GAUSS is the discreted kernel function
						temp_stixel[kde_val - 8 + k] +=
							W17_GAUSS[k];
					}
				}
			}
		}
		temp_frame.push_back(temp_stixel);
	}
	kde_frame_queue_.push(temp_frame);
}

/* finds and saves the position of peaks meeting certain conditions */
void StixelComponent::FindKdePeakPos(float delta_y) {
	auto &kde_frame = kde_frame_queue_.front();
	std::vector<std::vector<KdePeak>> kde_peak_frame;
	for (int i = 0; i < kde_frame.size(); i++) {
		// 1 for ascending and -1 for descending
		int prev_dir =
			kde_frame[i][1] > kde_frame[i][0] ? 1 : -1;
		std::vector<KdePeak> kde_peak;
		for (int j = 1; j < kde_width_ - 1; j++) {
			int temp_dir =
				kde_frame[i][j + 1] > kde_frame[i][j] ? 1 : -1;
			if (prev_dir == 1 && temp_dir == -1) {
				// y = yl * b * kw / k / dmax,
				// y is physical vertical length
				// yl is the y-coordinate value of kde; 
				// b is baseline; kw is kde_width; 
				// k is j here, or x-coordinate of kde;
				// dmax is disparity max, = disp_max * width_
				if (kde_frame[i][j] * baseline_ * kde_width_ /
					(j * disp_max_ * width_) > delta_y) {
                    /*auto disp_temp = j/kde_width_*disp_max_;
                    Point3D p_camera = GetCameraCoor(
                        disp_temp, i, height_/2);
                    Point3D p_world = CameraToWorldCoor(
                        scaled_disparity_frame_queue_.front().pos_camera_,
                        p_camera,
                        scaled_disparity_frame_queue_.front().angle_camera_);
                    KdePeak peak{p_world.x_, p_world.y_, j};*/
                    KdePeak peak {j};
                    // filter window
                    auto thh = kde_frame[i][j] * 0.85; // or 0.707 maybe
                    int jl = j, jr = j;
                    while(kde_frame[i][jl] > thh){
                        jl--;
                    }
                    while(kde_frame[i][jr] > thh){
                        jr++;
                    }
                    double delta_y_meter = 0.2; // 0.2m for window height
                    int delta_y_pix = static_cast<int>(
                        delta_y_meter*j/kde_width_*disp_max_/baseline_);
                    peak.SetWindow(jl, jr, delta_y_pix);
                    kde_peak.push_back(peak);
				}
			}
			prev_dir = temp_dir;
		}
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
| y
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
        FilterStatus stat{flag, pos};
        return stat; 
    }
}

/* Sliding-block filter */
void StixelComponent::DetectObject() {
	auto &scaled_disparity_frame =
		scaled_disparity_frame_queue_.front();
	auto &kde_peak_frame = kde_peak_frame_queue_.front();
	auto n_stixel = static_cast<int>(
        scaled_disparity_frame.data_.size());
    PillarFrame pillar_frame;
    // for each stixel in one frame
    for(auto stx_i=0; stx_i<kde_peak_frame.size(); stx_i++){
        std::vector<Pillar> pillar_col;
        BlockedIndex index {n_stixel};
        auto n_peak = kde_peak_frame[stx_i].size();
        // for each peak in one stixel
        for(auto peak_i=n_peak-1; peak_i>=0; peak_i--){
            Pillar pillar_temp;
            auto n_idx = index.size()-1;
            // for each unblocked segment
            for(auto idx=0; idx<n_idx; idx++){
                auto start = index[idx].second;
                auto end = index[idx+1].first;
                auto window_height = 
                    kde_peak_frame[stx_i][peak_i].windoe_height_;
                auto step_size = window_height/2;
                if(window_height < end - start){
                    double mean = kde_peak_frame[stx_i][peak_i].pos_
                        /kde_width_*disp_max_;
                    double min = kde_peak_frame[stx_i][peak_i].window_left_
                        /kde_width_*disp_max_;
                    double max = kde_peak_frame[stx_i][peak_i].window_right_
                        /kde_width_*disp_max_;
                    auto prev_stat = Filter(scaled_disparity_frame[stx_i], 
                            start, (end-start) % step_size, mean, min, max);
                    start += (end-start) % step_size;
                    int index_flag = 1;  // 1 for being seeking pillar head, 
                                            // -1 for being seeking pillar tai
                    while(start < end){
                        auto stat = Filter(scaled_disparity_frame[stx_i],
                        start, step_size, mean, min, max);
                        if(index_flag == 1){
                            if(prev_stat.flag_ == kNotCompliant 
                                && stat.flag_ == kCompliant){
                                auto p_camera = GetCameraCoor(
                                    mean, stx_i, prev_stat.value_);
                                auto p_world = CameraToWorldCoor(
                                    scaled_disparity_frame.pos_camera_,
                                    p_camera,
                                    scaled_disparity_frame.angle_camera_);
                                pillar_temp.SetPoint(p_world);
                                index_flag = -1;
                            }
                        }else if(index_flag == -1){
                            if(prev_stat.flag_ == kCompliant 
                                && stat.flag_ == kNotCompliant){
                                auto p_camera = GetCameraCoor(
                                    mean, stx_i, stat.value_);
                                auto p_world = CameraToWorldCoor(
                                    scaled_disparity_frame.pos_camera_,
                                    p_camera,
                                    scaled_disparity_frame.angle_camera_);
                                pillar_temp.SetZ2(p_world.z_);
                                pillar_col.push_back(pillar_temp);
                                index_flag = 1;
                            }
                        }
                        prev_stat = stat;
                        start += step_size;
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
		&StixelComponent::Stixel, this };
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