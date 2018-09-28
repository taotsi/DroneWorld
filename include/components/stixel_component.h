#pragma once
#include "components\base_component.h"

class StixelComponent :public BaseComponent {
public:
	/* methods */
	StixelComponent(std::queue<ImageResponse>* disparity_retreived);
	~StixelComponent();
	void Update(double DeltaTime);
	void Begin();
	std::vector<double> GetKde();
	/* data */
	std::queue<ImageResponse>* disparity_retreived_;
	std::queue<std::vector<std::vector<double>>> stixel_scaled_frame_queue_;
	// kde for disparity
	std::queue<std::vector<std::vector<double>>> kde_frame_queue_;
	std::queue<std::vector<std::vector<int>>> kde_peak_pos_frame_queue_;
private:
	/* methods */
	void Stixel();
	void Kde();
	void FindKdePeakPos(float delta_y = 0.5);
	void DetectObject();
	void Behave();
	std::thread thread_handle_;
	/* data */
	int width_ = 640;
	int height_ = 320;
	int kde_width_ = 1000;
	int stixel_width_ = 7;
	float base_line_ = 0.25;
	float fov_ = 90;
	// normalized disparity
	float disp_max_ = 0.25f;   // 0.5m
	float disp_min_ = 0.00625f; // 20m
	std::vector<double> W17_GAUSS;
};