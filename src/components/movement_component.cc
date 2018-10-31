#include "components/movement_component.h"

namespace droneworld{

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
    client_.takeoffAsync(4)->waitOnLastTask();
    client_.hoverAsync()->waitOnLastTask();
    
    std::vector<Vector3r> path{Vector3r(0, 0, -7)};
    client_.moveOnPathAsync(path, speed_, 300, DrivetrainType::ForwardOnly, 
        YawMode(false, 0), -1, 0)->waitOnLastTask();
    
    move_thread_ = std::thread{&MovementComponent::MoveThreadMain, this};
    move_thread_.detach();
}

void MovementComponent::Update(double DeltaTime){
    
}
// The movement management is put in a seperated thread, instead of Update(). MoveThreadMain() keeps checking out if there is a path the drone should follow.
void MovementComponent::MoveThreadMain(){
    while(is_on_){
        if(!path_to_go_.empty()){
            auto point_to_go = path_to_go_.front();
            client_.moveToPositionAsync(
                point_to_go[1], point_to_go[0], -point_to_go[2], speed_, 60, 
                DrivetrainType::ForwardOnly, YawMode(false, 0), -1, 0)
                ->waitOnLastTask();
            path_to_go_.pop();
        }
    }
}
// Adds a target point to the path the drone would follow, it will fly there after all the past target points have been reached.
void MovementComponent::AddPathPoint(const std::vector<float> &point){
    if(point.size() == 3){
        path_to_go_.push(point);
    }else{
        std::cout << "failed to add new point to path\n";
    }
}
// Adds a series of target points for the drone to follow.
void MovementComponent::AddPath(const std::vector<std::vector<float>> &path){
    auto n_point = path.size();
    for(auto i=0; i<n_point; i++){
        AddPathPoint(path[i]);
    }
}
void MovementComponent::ResetPath(
    const std::queue<std::vector<float>> &new_path){
    path_to_go_ = new_path;
}

void MovementComponent::SetSpeed(float speed){
    speed_ = speed;
}

void MovementComponent::MoveTest(){
    float z = -2, vel = 2;
    std::vector<Vector3r> path{Vector3r(0, 0, z), Vector3r(0, -8, z),Vector3r(-18, -8, z)};
    client_.moveOnPathAsync(path, vel, 300, DrivetrainType::ForwardOnly, 
        YawMode(false, 0), -1, 0)->waitOnLastTask();
    client_.hoverAsync()->waitOnLastTask();
    std::this_thread::sleep_for(std::chrono::seconds(5));
}
}