#pragma once
#include <memory>
#include "drone.h"
#include "world.h"

namespace droneworld{

Drone::Drone(std::string name)
  : name_(name) {
	movement_ = std::make_unique<MovementComponent>();
	image_record_ = 
		std::make_unique<ImageRecordComponent>();
	stixel_ = std::make_unique<StixelComponent>(
		&(image_record_->disparity_retreived_),
        640, 480, 1.91986218);
    cluster_ = std::make_unique<PillarClusterComponent>(
        &(stixel_->pillar_frame_queue_));
    World::drone_list_.insert(
		std::pair<std::string, Drone*>(name_, this));
}

Drone::~Drone() {
    World::RemoveDrone(name_);
    movement_->Stop();
    image_record_->Stop();
    stixel_->Stop();
    cluster_->Stop();
    std::cout << name_ << " removed\n";
}

void Drone::Begin() {
    movement_->Begin();
    // image_record_->Begin();
	// stixel_->Begin();
    // cluster_->Begin();
}

void Drone::Update(double DeltaTime) {
    movement_->Update(DeltaTime);
    // image_record_->Update(DeltaTime);
	// stixel_->Update(DeltaTime);
    // cluster_->Update(DeltaTime);
}
}