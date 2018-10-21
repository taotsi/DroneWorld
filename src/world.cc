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
    std::string drone1_str{"drone1"};
    selected_drone_ = drone1_str;
    drone_names_.insert(drone1_str);
    auto drone1 = std::make_unique<Drone>(drone1_str);
    drone_list_.insert(
        std::pair<std::string, std::unique_ptr<Drone>>(drone1_str, drone1));
}

void World::RemoveDrone(std::string &name){
    World::drone_list_.erase(name);
    auto drone_name_ptr = std::find(drone_names_.begin(), drone_names_.end(), name);
    drone_names_.erase(drone_name_ptr);
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
    std::stringstream ss{msg};
    std::string first;
    ss >> first;
    switch(first){
        case "ls":{
            CmdLs();
            break;
        }
        case "select":{
            CmdSelect();
            break;
        }
        case "go":{
            break;
        }
        case "rec":{
            break;
        }
        default:{
            std::cout << "can't identify command \"" << msg << "\"\n";
        }
    }
}

void World::CmdLs(std::stringstream &ss){
    if(ss.good()){
        std::cout << "just \"ls\" is enough, no more arguments!\n";
    }
    for(auto it : drone_names_){
        std::cout << it << "\t";
    }
    std::cout << "\n";
}
void World::CmdSelect(std::stringstream &ss){
    std::string name;
    ss >> name;
    if(ss.good()){
        std::cout << "only one drone can be selected at a time\n";
    }
    auto name_ptr = std::find(drone_names_.begin(), drone_names_.end(), name);
    if(name_ptr == drone_names_.end()){
        std::cout << "no such a drone named " << name << " \n";
        return;
    }
    selected_drone_ = name;
}
void World::CmdGo(std::stringstream &ss){
    std::vector<double> xyz_vec;
    double temp;
    while(ss.good()){
        ss >> temp;
        xyz_vec.push_back(temp);
    }
    if(xyz_vec.size() != 3){
        std::cout << "just input 3 arguments, for point x, y, z you'd like "
            << selected_drone_ << " to go\n";
        return;
    }
}
void World::CmdRec(std::stringstream &ss){
    
}

std::map<std::string, Drone*> World::drone_list_ = 
	std::map<std::string, Drone*>();
