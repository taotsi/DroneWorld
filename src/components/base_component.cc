#include "components/base_component.h"

namespace droneworld{

BaseComponent::BaseComponent() {
    Begin();
}

BaseComponent::~BaseComponent() {
    /*if (thread_.joinable()) {
        thread_.join();
    }*/
}

void BaseComponent::Begin() {
}
}