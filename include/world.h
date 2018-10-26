#pragma once

#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <random>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <memory>
#include <utility>
#include <algorithm>
#include "drone.h"

namespace droneworld{

class World {
public:
    static World& Instance();
    ~World();
    void Loop();
    friend class Drone;
    friend class RpcServer;
private:
    /* data */
    std::string selected_drone_;
    static std::set<std::string> drone_names_;
    static std::map<std::string, std::unique_ptr<Drone>> drone_list_;
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
    static void RemoveDrone(std::string &name);
    void InputThreadMain();
    inline void ProcessInput(std::string &msg);
    void CmdLs(std::stringstream &ss);
    void CmdSelect(std::stringstream &ss);
    void CmdGo(std::stringstream &ss);
    void CmdSetspd(std::stringstream &ss);
    void CmdRec(std::stringstream &ss);
    void CmdKde(std::stringstream &ss);
};
}