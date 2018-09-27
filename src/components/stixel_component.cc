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

void StixelComponent::Behave() {
	is_busy_ = true;
	thread_handle_ = std::thread{ 
		&StixelComponent::Stixel, this };
}

/* for rpclib server */
std::vector<float> StixelComponent::GetKde() {
	return kde_frame_queue_.front()[45];
}