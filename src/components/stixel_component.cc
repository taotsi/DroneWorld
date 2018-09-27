#include "components\stixel_component.h"
#include <algorithm>
#include <cmath>

StixelComponent::StixelComponent
	(std::queue<ImageResponse>* disparity_retreived)
	:disparity_retreived_(disparity_retreived) {
}
StixelComponent::~StixelComponent() {

}
void StixelComponent::Begin() {
	Kde(); // test
}
void StixelComponent::Update(double DeltaTime) {

}

void StixelComponent::Stixel() {
	Kde();
	FindKdePeakPos();
}

void StixelComponent::Kde() {
	/* get a frame of disparity if available */
	if (!disparity_retreived_->empty()) {
		auto frame_raw = disparity_retreived_->front();
		disparity_retreived_->pop();
		std::vector<std::vector<float>> frame_scaled;
		for (int i = 4; i < width_; i += stixel_width_) {
			std::vector<float> pix_col;
			for (int j = 0; j < height_; j++) {
				float pix_val = frame_raw.
					image_data_float[j*width_ + i];
				pix_col.push_back(pix_val);
			}
			frame_scaled.push_back(pix_col);
		}
		/* kernel density estimate */
		std::vector<std::vector<float>> temp_frame;
		for (int i = 0; i < frame_scaled.size(); i++) {
			std::vector<float> temp_stixel(kde_width_, 0);
			for (int j = 0; j < height_; j++) {
				if (frame_scaled[i][j] <= disp_max_ &&
					frame_scaled[i][j] >= disp_min_){
					int kde_val = static_cast<int>(
						frame_scaled[i][j] * kde_width_ /disp_max_);
					for (int k = 0; k < 17; k++) {
						if (kde_val - 8 >= 0 &&
							kde_val + 8 <= kde_width_ - 1) {
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
}

/* returns the position of peaks that meets certain conditions */
void StixelComponent::FindKdePeakPos(float delta_y = 0.5) {
	if (!kde_frame_queue_.empty()) {
		auto kde_frame = kde_frame_queue_.front();
		kde_frame_queue_.pop();
		std::vector<std::vector<int>> kde_peak_pos_frame;
		for (int i = 0; i < kde_frame.size(); i++) {
			int prev_dir = 
				kde_frame[i][1] > kde_frame[i][0] ? 1 : -1;
			std::vector<int> kde_peak_pos;
			for (int j = 1; j < kde_width_-1; j++) {
				int temp_dir = 
					kde_frame[i][j + 1] > kde_frame[i][j] ? 1 : -1;
				if (prev_dir == 1 && temp_dir == -1) {
					// y = yl * b * kw / k / dmax,
					// y is physical vertical length
					// yl is pixel quantity, aka y-coordinate of kde; 
					// b is baseline;
					// kw is kde_width; 
					// k is j here, or x-coordinate of kde;
					// dmax is disparity max
					if (kde_frame[i][j] * base_line_ * kde_width_ /
						j / disp_max_ > 0.5) {
						kde_peak_pos.push_back(j);
					}
				}
				prev_dir = temp_dir;
			}
			kde_peak_pos_frame.push_back(kde_peak_pos);
		}
		kde_peak_pos_frame_queue_.push(kde_peak_pos_frame);
	}
}

void StixelComponent::Behave() {
	is_busy_ = true;
	thread_handle_ = std::thread{ 
		&StixelComponent::Stixel, this };
}

/* for rpclib server */
std::vector<float> StixelComponent::GetKde() {
	return kde_frame_queue_.front()[45];
}