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
		// POP	disparity_retreived_; 
		// PUSH stixel_scaled_frame_queue_
		// PUSH kde_frame_queue_
		Kde();				
	}
	if (!kde_frame_queue_.empty()) {
		// PUSH kde_peak_pos_frame_queue_
		FindKdePeakPos(0.5);
	}
	if (!kde_frame_queue_.empty()
		&& !stixel_scaled_frame_queue_.empty()
		&& !kde_peak_pos_frame_queue_.empty()) {
		// POP	kde_frame_queue_
		// POP	stixel_scaled_frame_queue_
		// POP	kde_peak_pos_frame_queue_
		// PUSH ??
		DetectObject();
	}
  				
}

/* gets kde and saves them */
// do NOT call this alone, it's put in Stixel() in order
void StixelComponent::Kde() {
	/* get a frame of disparity if available */
	// TODO: if roll != 0, correction needs to be done
	// won't check out if disparity_retreived_ is empty
	auto frame_raw = disparity_retreived_->front();
	disparity_retreived_->pop();
	std::vector<std::vector<double>> frame_scaled;
	int i_start = stixel_width_ / 2;
	for (int i = i_start; i < width_; i += stixel_width_) {
		std::vector<double> pix_col;
		for (int j = 0; j < height_; j++) {
			double pix_val = frame_raw.
				image_data_float[j*width_ + i];
			pix_col.push_back(pix_val);
		}
		frame_scaled.push_back(pix_col);
	}
	stixel_scaled_frame_queue_.push(frame_scaled);
	/* kernel density estimate */
	std::vector<std::vector<double>> temp_frame;
	for (int i = 0; i < frame_scaled.size(); i++) {
		std::vector<double> temp_stixel(kde_width_, 0);
		for (int j = 0; j < height_; j++) {
			if (frame_scaled[i][j] <= disp_max_ &&
				frame_scaled[i][j] >= disp_min_) {
				int kde_val = static_cast<int>(
					frame_scaled[i][j] * kde_width_ / disp_max_);
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
// do NOT call this alone, it's put in Stixel() in order
void StixelComponent::FindKdePeakPos(float delta_y) {
	// won't check out if kde_frame_queue_ is empty
	auto kde_frame = kde_frame_queue_.front();
	std::vector<std::vector<int>> kde_peak_pos_frame;
	for (int i = 0; i < kde_frame.size(); i++) {
		// 1 for ascending and -1 for descending
		int prev_dir =
			kde_frame[i][1] > kde_frame[i][0] ? 1 : -1;
		std::vector<int> kde_peak_pos;
		for (int j = 1; j < kde_width_ - 1; j++) {
			int temp_dir =
				kde_frame[i][j + 1] > kde_frame[i][j] ? 1 : -1;
			if (prev_dir == 1 && temp_dir == -1) {
				// y = yl * b * kw / k / dmax,
				// y is physical vertical length
				// yl is pixel quantity, aka y-coordinate of kde; 
				// b is baseline; kw is kde_width; 
				// k is j here, or x-coordinate of kde;
				// dmax is disparity max, = disp_max * width_
				if (kde_frame[i][j] * baseline_ * kde_width_ /
					(j * disp_max_ * width_) > delta_y) {
					kde_peak_pos.push_back(j);
				}
			}
			prev_dir = temp_dir;
		}
		if (kde_peak_pos.empty()) {
			kde_peak_pos_frame.push_back(std::vector<int>());
		}
		else {
			kde_peak_pos_frame.push_back(kde_peak_pos);
		}
	}
	kde_peak_pos_frame_queue_.push(kde_peak_pos_frame);
}

/* Sliding-block filter */
// do NOT call this alone, it's put in Stixel() in order
void StixelComponent::DetectObject() {
	// won't check out if either stixel_scaled_frame_queue_
	// or kde_peak_pos_frame_queue_ is empty
	auto stixel_scaled_frame = stixel_scaled_frame_queue_.front();
	stixel_scaled_frame_queue_.pop();
	auto kde_peak_pos_frame = kde_peak_pos_frame_queue_.front();
	kde_peak_pos_frame_queue_.pop();
	auto n_stixel = stixel_scaled_frame.size();
	for (int i = 0; i < n_stixel; i++) {

	}
}

/*
our coordinate system:
z
| y
|/
0----x
yet in Airsim, +X is North, +Y is East and +Z is Down
ours is also different with dual-camera's coordinate
*/
Point3D StixelComponent::TransformAirsimCoor(
	double x, double y, double z) {
	return Point3D{ y, x, -z };
}

Point3D StixelComponent::GetCameraCoor(
	double disp_normalized, int x_pixel_scaled, int y_pixel) {
	Point3D p_camera;
	double focus = baseline_ / (disp_normalized * 2 * tan(fov_/2));
	p_camera.y_ = focus * baseline_ / (disp_normalized * width_);
	p_camera.z_ = (y_pixel - height_/2) * p_camera.y_ / focus;
	int x_pixel = stixel_width_ * (x_pixel_scaled) + stixel_width_/2;
	p_camera.x_ = (x_pixel - width_/2) * p_camera.y_ / focus;
	return p_camera;
}

/* returns world coordinate position */
// use rad instead of degree
Point3D StixelComponent::CameraToWorldCoor(
	Point3D camera_pos, Point3D p_camera, EulerAngle angle) {
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
			+ sin_theta*sin_phi*cos_omega);
	p_world.y_ =
		p_camera.x_ * (sin_theta * cos_omega 
			+ sin_omega * cos_theta*sin_phi)
		+ p_camera.y_ * (cos_theta * cos_phi)
		+ p_camera.z_ * (- sin_theta * sin_omega 
			+ cos_theta * sin_phi * cos_omega);
	p_world.y_ =
		p_camera.x_ * (sin_omega * cos_phi)
		- p_camera.y_ * (sin_phi)
		+ p_camera.z_ * (cos_phi * cos_omega);
	// translation
	p_world.x_ += camera_pos.x_;
	p_world.y_ += camera_pos.y_;
	p_world.z_ += camera_pos.z_;
	return p_world;
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