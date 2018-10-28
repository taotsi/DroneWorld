#include "world.h"

namespace droneworld{

World::World() {
    SpawnDrones();
}

World::~World() {
    std::cout << "world destroyed\n";
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
    std::cout << "world init ready\n";
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
    std::string drone1_name{"drone1"};
    selected_drone_ = drone1_name;
    drone_names_.insert(drone1_name);
    std::unique_ptr<Drone> drone1(new Drone{drone1_name});
    drone_list_[drone1_name] = std::move(drone1);
}

void World::RemoveDrone(std::string &name){
    World::drone_list_.erase(name);
    auto drone_name_ptr = std::find(
        drone_names_.begin(), drone_names_.end(), name);
    drone_names_.erase(drone_name_ptr);
}

/* Input Handling */
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
    if(first == "ls"){
        CmdLs(ss);
    }else
    if(first == "select"){
        CmdSelect(ss);
    }else
    if(first == "go"){
        CmdGo(ss);
    }else 
    if(first == "rec"){
        // TODO
    }else
    if(first == "setspd"){
        CmdSetspd(ss);
    }else
    if(first == "kde"){
        CmdKde(ss);
    }else
    if(first == "disp"){
        CmdDisp(ss);
    }
    else{
        std::cout << "can't identify command \"" << msg << "\"\n";
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
    std::vector<float> xyz_vec;
    float temp;
    while(ss.good()){
        ss >> temp;
        xyz_vec.push_back(temp);
    }
    if(xyz_vec.size() == 3){
        drone_list_[selected_drone_]->movement_->AddPathPoint(xyz_vec);
    }else{
        std::cout << "just input 3 arguments, for point x, y, z you'd like "
            << selected_drone_ << " to go\n";
        return;
    }
}
void World::CmdSetspd(std::stringstream &ss){
    if(ss.good()){
        float speed;
        ss >> speed;
        drone_list_[selected_drone_]->movement_->SetSpeed(speed);
    }else{
        std::cout << "need one argument, like setspd [speed_val]";
    }
}
void World::CmdDisp(std::stringstream &ss){
    if(ss.good()){
        int col;
        ss >> col;
        drone_list_[selected_drone_]->stixel_->PrintDisp(col);
    }else{
        std::cout << "disp [col_index_value], like kde 45\n";
    }
}
void World::CmdKde(std::stringstream &ss){
    if(ss.good()){
        int col;
        ss >> col;
        drone_list_[selected_drone_]->stixel_->PrintKde(col);
    }else{
        std::cout << "kde [col_index_value], like kde 45\n";
    }
}
void World::CmdRec(std::stringstream &ss){
    
}

std::map<std::string,  std::unique_ptr<Drone>> World::drone_list_ = 
	std::map<std::string,  std::unique_ptr<Drone>>();
std::set<std::string> World::drone_names_ = 
    std::set<std::string>();

} // namespace droneworld
