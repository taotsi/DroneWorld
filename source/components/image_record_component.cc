#include "components/image_record_component.h"

ImageRecordComponent::ImageRecordComponent() {
}

void ImageRecordComponent::ThreadMain() {
    for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Record tick\n";
    }
}