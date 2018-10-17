#include "components/movement_component.h"

MovementComponent::MovementComponent() { 
}

void MovementComponent::Begin() {
    std::cout << "\n---- movement client ---- \n";
    client_.confirmConnection();
    MoveTest();
    // std::thread thread_{ &MovementComponent::MoveTest, this };
    // thread_.detach();
}

void MovementComponent::MoveTest(){
    client_.enableApiControl(true);
    client_.armDisarm(true);
    client_.takeoffAsync(5)->waitOnLastTask();
    client_.hoverAsync()->waitOnLastTask();
    
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    float z = -10;
    float vel = 20;
    std::vector<Vector3r> path{Vector3r(0, 0, z), Vector3r(200, 0, z),Vector3r(200, 50, z)};
    //std::vector<Vector3r> path{Vector3r(0, 0, -5)};
    client_.moveOnPathAsync(path, vel, 300, 
        DrivetrainType::ForwardOnly, YawMode(false, 0), -1, 0)
        ->waitOnLastTask();
    
    client_.hoverAsync()->waitOnLastTask();
    //client_.landAsync()->waitOnLastTask();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    //client_.armDisarm(false);
    //client_.enableApiControl(false);
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