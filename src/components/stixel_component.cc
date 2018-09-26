#include "components\stixel_component.h"
#include <algorithm>
#include <cmath>

StixelComponent::StixelComponent
	(std::queue<ImageResponse>* disparity_retreived)
	:disparity_retreived_(disparity_retreived) {
}
StixelComponent::~StixelComponent() {

}
void StixelComponent::Update(double DeltaTime) {

}
void StixelComponent::Begin() {

}
void StixelComponent::Stixel() {

}
void StixelComponent::Kde() {
	// get a frame of disparity if available
	std::vector<std::vector<float>> frame_scaled;
	if (!disparity_retreived_->empty()) {
		auto frame_raw = disparity_retreived_->front();
		//width = frame_raw.width; // fixed value 640
		//height = frame_raw.height;  // fixed value 480
		for (int i = 4; i < width; i += stixel_width) {
			std::vector<float> pix_col;
			for (int j = 0; j < height; j++) {
				float pix_val = frame_raw.
					image_data_float[j*width + i];
				pix_col.push_back(pix_val);
			}
			frame_scaled.push_back(pix_col);
		}
		disparity_retreived_->pop();
	}
	// kernel density estimate
	std::vector<std::vector<float>> temp_frame;
	for (int i = 0; i < frame_scaled.size(); i++) {
		std::vector<float> temp_stixel(kde_width_, 0);
		for (int j = 0; j < height; j++) {
			if (frame_scaled[i][j] <= disp_max &&
				frame_scaled[i][j] >= disp_min){
				int kde_val = static_cast<int>(
					frame_scaled[i][j] * kde_width_ / disp_max);
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
	kde_frame.push(temp_frame);
}
std::vector<float> StixelComponent::GetKde(int pos = 320) {
	return kde_frame.front()[pos];
}
void StixelComponent::Behave() {
	is_busy_ = true;
	thread_handle_ = std::thread{ 
		&StixelComponent::Stixel, this };
}