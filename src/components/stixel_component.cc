#include "components\stixel_component.h"

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
void StixelComponent::Behave() {
	is_busy_ = true;
	thread_handle_ = std::thread{ 
		&StixelComponent::Stixel, this };
}