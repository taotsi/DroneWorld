#pragma once

#include "drone.h"
#include "world.h"

Drone::Drone(std::string name)
  : name_(name) {
    World::drone_list_.insert(std::pair<std::string, Drone*>(name_, this));
}

Drone::~Drone() {
    World::drone_list_.erase(name_);
}

void Drone::Begin() {
    Movement_.Begin();
    ImageRecord_.Begin();
    ImageProcess_.Begin();
    Communication_.Begin();
}

void Drone::Update(double DeltaTime) {
    std::cout << "drone update\n";
    Movement_.Update(DeltaTime);
    ImageRecord_.Update(DeltaTime);
    ImageProcess_.Update(DeltaTime);
    Communication_.Update(DeltaTime);
}