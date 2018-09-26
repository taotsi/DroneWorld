#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <random>
#include <vector>
#include <map>
#include <set>
#include "drone.h"

class World {
public:
    static World& Instance();
	World();
    virtual ~World();
    void Loop();
    friend class Drone;
	/* data */
	static std::map<std::string, Drone*> drone_list_;
	void test();

private:
	
    void ProcessInput();
    void Begin();
    void Update(double DeltaTime);
    void SpawnDrones();

    bool isDone = false;
    const std::chrono::duration<double, std::ratio<1, 1000>> MS_PER_UPDATE { 50.0 }; // in milliseconds
    friend class Drone;
    
};