#include "components/movement_component.h"

MovementComponent::MovementComponent(){ 
    
}
MovementComponent::~MovementComponent(){
    client_.landAsync()->waitOnLastTask();
    client_.armDisarm(false);
    client_.enableApiControl(false);
}
void MovementComponent::Begin(){
    Start();
    std::cout << "\n---- movement client ---- \n";
    client_.confirmConnection();
    client_.enableApiControl(true);
    client_.armDisarm(true);
    client_.takeoffAsync(5)->waitOnLastTask();
    client_.hoverAsync()->waitOnLastTask();
    // MoveTest();
    move_thread_ = std::thread{&MovementComponent::MoveThreadMain, this};
    thread_.detach();
}

void Update(double DeltaTime){
    if(path_to_go_.empty()){
        is_busy_ = false;
    }else{
        is_busy_ = true;
    }
}

void MovementComponent::MoveThreadMain(){
    float speead = 5;
    while(is_on_){
        if(is_busy_){
            auto point_to_go = path_to_go_.front();
            client_.moveToPositionAsync(
                point_to_go[1], point_to_go[0], -point_to_go[2], speed, 300, 
                DrivetrainType::ForwardOnly, YawMode(false, 0), -1, 0)
                ->waitOnLastTask();
        }
    }
}
void MovementComponent::AddPathPoint(const std::vector<double> &point){
    if(point.size() == 3){
        path_to_go_.push(point);
    }else{
        std::cout << "failed to add new point to path\n";
    }
}
void MovementComponent::ResetPath(
    const std::queue<std::vector<double>> &new_path){
    path_to_go_ = new_path;
}

void MovementComponent::MoveTest(){
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    float z = -10;
    float vel = 20;
    std::vector<Vector3r> path{Vector3r(0, 0, z), Vector3r(200, 0, z),Vector3r(200, 50, z)};
    //std::vector<Vector3r> path{Vector3r(0, 0, -5)};
    client_.moveOnPathAsync(path, vel, 300, 
        DrivetrainType::ForwardOnly, YawMode(false, 0), -1, 0)
        ->waitOnLastTask();
    client_.hoverAsync()->waitOnLastTask();
    std::this_thread::sleep_for(std::chrono::seconds(5));
}

void MovementComponent::BehaveForcibly() {
    is_busy_ = true;

	is_busy_ = false;
}

bool MovementComponent::BehaveSafely() {
    if (is_busy_) {
        std::cout << "Still on another mission!\n";
    } else {
		is_busy_ = true;

    }
    return is_busy_;
}