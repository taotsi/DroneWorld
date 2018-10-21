#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <random>
#include <vector>
#include <map>
#include <queue>
#include "drone.h"

class World {
public:
    static World& Instance();
    ~World();
    void Loop();	
	void test();
private:
	friend class Drone;
    friend class RpcServer;
    /* data */
    static std::map<std::string, Drone*> drone_list_;
    const std::chrono::duration<double, std::ratio<1, 1000>>
        MS_PER_UPDATE { 50.0 }; // in milliseconds
    bool is_on_ = false;
    std::queue<std::string> msg_queue_;
    std::thread input_thread_;
    
    /* methods */
    World();
    void Begin();
    void Update(double DeltaTime);
    void SpawnDrones();
    void InputThreadMain();
    void ProcessInput(std::string &msg);
};