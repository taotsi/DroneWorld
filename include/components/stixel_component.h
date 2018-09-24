#pragma once
#include "components\base_component.h"

class StixelComponent :public BaseComponent {
public:
	StixelComponent(std::queue<ImageResponse>* disparity_retreived);
	~StixelComponent();
	void Update(double DeltaTime);
	void Begin();
	std::queue<ImageResponse>* disparity_retreived_;
private:
	void Stixel();
	void Behave();
	std::thread thread_handle_;
};