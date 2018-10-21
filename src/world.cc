#include "world.h"

World::World() {
    SpawnDrones();
}

World::~World() {
}

World& World::Instance() {
    static World* instance = new World();
    return *instance;
}

void World::Begin() {
    is_on_ = true;
    input_thread_ = std::thread{&World::InputThreadMain, this};
    input_thread_.detach();
    
    if (drone_list_.empty()) {
        std::cout << "No drones in drone_map_!\n";
    } else {
        for (const auto &itr : drone_list_) {
            itr.second->Begin();
        }
    }
}

void World::Update(double DeltaTime) {
    if (drone_list_.empty()) {
        std::cout << "No drones in drone_map_!\n";
    } else {
        for (const auto &itr : drone_list_) {
            itr.second->Update(DeltaTime);
        }
    }
}

void World::Loop() {
    Begin();

    auto previous = std::chrono::system_clock::now();
    while (is_on_) {
        auto current = std::chrono::system_clock::now();
        std::chrono::duration<double, std::ratio<1, 1000>> elapsed = current - previous;
        previous = current;

        Update(elapsed.count());
    }
    std::cout << "****** world exit ******\n";
}

void World::SpawnDrones() {
    Drone* drone1 = new Drone("drone1");
    drone_list_.insert(std::pair<std::string, Drone*>("drone1", drone1));
}

void World::InputThreadMain(){
    std::string line;
    while(is_on_){
        if(!msg_queue_.empty()){
            auto &msg = msg_queue_.front();
            if(msg != "exit"){
                ProcessInput(msg);
                msg_queue_.pop();
            }else{
                is_on_ = false;
                break;
            }
        }
        if(getline(std::cin, line)){
            msg_queue_.push(line);
        }
    }
    std::cout << "*** input handler exit ***\n";
}

void World::ProcessInput(std::string &msg) {
    std::cout << msg << "\n";
}

std::map<std::string, Drone*> World::drone_list_ = 
	std::map<std::string, Drone*>();
