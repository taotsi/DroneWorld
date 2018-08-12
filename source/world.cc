#include "world.h"

World& World::Instance() {
    static World* instance = new World();
    return *instance;
}

World::~World() {
}

void World::Loop() {
    Begin();
    auto previous = std::chrono::system_clock::now();
    while (!isDone) {
        auto current = std::chrono::system_clock::now();
        std::chrono::duration<double, std::ratio<1, 1000>> elapsed = current - previous;
        previous = current;

        ProcessInput();
        Update(elapsed.count());
    }
}

World::World() {
    SpawnDrones();
}

void World::ProcessInput() {
    //std::cout << "process input\n";
}

void World::Begin() {
    if (drone_list_.empty()) {
        std::cout << "No drones in drone_map_!\n";
    }
    else {
        for (const auto &itr : drone_list_) {
            itr.second->Begin();
        }
    }
}

void World::Update(double DeltaTime) {
    //int min = 200;
    //int max = 800;
    //std::random_device rd;     // only used once to initialise (seed) engine
    //std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
    //std::uniform_int_distribution<int> uni(min, max); // guaranteed unbiased
    //int randint = uni(rng);

    //std::this_thread::sleep_for(std::chrono::milliseconds(randint));
    //std::cout << randint << "ms, update\n";
    if (drone_list_.empty()) {
        std::cout << "No drones in drone_map_!\n";
    } else {
        for (const auto &itr : drone_list_) {
            itr.second->Update(DeltaTime);
        }
    }
}

void World::SpawnDrones() {
    Drone* drone1 = new Drone("drone1");
    drone_list_.insert(std::pair<std::string, Drone*>("drone1", drone1));
}

std::map<std::string, Drone*> World::drone_list_ = std::map<std::string, Drone*>();