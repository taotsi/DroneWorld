#include "components/base_component.h"

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