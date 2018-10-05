#pragma once
#include <memory>
#include "drone.h"
#include "world.h"

Drone::Drone(std::string name)
  : name_(name) {
	movement_ = std::make_unique<MovementComponent>();
	image_record_ = 
		std::make_unique<ImageRecordComponent>();
	stixel_ = std::make_unique<StixelComponent>(
		&(image_record_->disparity_retreived_));
    cluster_ = std::make_unique<PillarClusterComponent>(
        &(stixel_->pillar_frame_queue_));
    World::drone_list_.insert(
		std::pair<std::string, Drone*>(name_, this));
}

Drone::~Drone() {
    World::drone_list_.erase(name_);
}

void Drone::Begin() {
    movement_->Begin();
    image_record_->Begin();
	stixel_->Begin();
    cluster_->Begin();
}

void Drone::Update(double DeltaTime) {
    //std::cout << "drone update\n";
    movement_->Update(DeltaTime);
    image_record_->Update(DeltaTime);
	stixel_->Update(DeltaTime);
    cluster_->Update(DeltaTime);
}