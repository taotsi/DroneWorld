#include "components/movement_component.h"

MovementComponent::MovementComponent() { 
}

void MovementComponent::Begin() {
    //std::thread thread_{ &MovementComponent::ThreadMain, this };
    //thread_.detach();
}

void MovementComponent::ThreadMain() {
    client_.confirmConnection();
    client_.enableApiControl(true);
    client_.armDisarm(true);
    client_.takeoffAsync(5)->waitOnLastTask();
    client_.hoverAsync()->waitOnLastTask();
    std::this_thread::sleep_for(std::chrono::duration<int>(5));

    client_.landAsync()->waitOnLastTask();
    client_.armDisarm(false);
    client_.enableApiControl(false);
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