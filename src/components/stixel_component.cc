#include "components\stixel_component.h"
#include "math.h"

StixelComponent::StixelComponent(std::queue<ImageResponse>* disparity_retreived)
  :disparity_retreived_(disparity_retreived){

}
StixelComponent::~StixelComponent(){

}
void StixelComponent::Update(double DeltaTime) {

}
void StixelComponent::Begin() {

}
void StixelComponent::Stixel() {

}
void StixelComponent::Kde() {
	const int stixel_width = 7;
	std::vector<std::vector<float>> frame_scaled;
	int width, height;
	if (!disparity_retreived_->empty()) {
		for (int i = 4; i < width; i += stixel_width) {
			std::vector<float> pix_col;
			for (int j = 0; j < height; j++) {
				float pix_val = disparity_retreived_->back().image_data_float[j*width + i];
				pix_col.push_back(pix_val);
			}
			frame_scaled.push_back(pix_col);
		}
		disparity_retreived_->pop();
	}

}
void StixelComponent::Behave() {
	is_busy_ = true;
	thread_handle_ = std::thread{ 
		&StixelComponent::Stixel, this };
}