#pragma once
#include "components\base_component.h"

class StixelComponent :public BaseComponent {
public:
	/* methods */
	StixelComponent(std::queue<ImageResponse>* disparity_retreived);
	~StixelComponent();
	void Update(double DeltaTime);
	void Begin();
	std::vector<float> GetKde(int pos);
	int test() { return 42; };
	/* data */
	std::queue<ImageResponse>* disparity_retreived_;
	std::queue<std::vector<std::vector<float>>> kde_frame;
private:
	/* methods */
	void Stixel();
	void Kde();
	void Behave();
	std::thread thread_handle_;
	/* data */
	int width = 640;
	int height = 480;
	int kde_width_ = 1000;
	int stixel_width = 7;
	float base = 0.25;
	float fov = 90;
	// normalized disparity
	float disp_max = 0.25f;   // 0.5m
	float disp_min = 0.0625f; // 20m


	std::vector<float> W17_GAUSS =
	{ 0.0561f, 0.1103f, 0.1979f, 0.3247f, 0.4868f, 0.6670f,
	  0.8354f, 0.9561f, 1.0001f, 0.9561f, 0.8354f, 0.6670f,
	  0.4868f, 0.3247f, 0.1979f, 0.1103f, 0.0561f };
};