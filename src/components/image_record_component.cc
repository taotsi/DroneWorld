#include "components/image_record_component.h"
#include <experimental/filesystem>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>

namespace fs = std::experimental::filesystem;

ImageRecordComponent::ImageRecordComponent(){
}

ImageRecordComponent::~ImageRecordComponent() {
    std::cout << "ImageRecordComponent destroyed\n";
    is_busy_ = false;
    // TODO: wait until the thread associated 
    //       with thread_handle_  has stopped. 
}

void ImageRecordComponent::Begin() {
    Behave();
}

void ImageRecordComponent::Record(bool save_as_file=false) {
    client_.confirmConnection();
    typedef common_utils::FileSystem FileSystem;

    fs::path data_dir = ".\\DataSet\\Disparity";
    fs::create_directories(data_dir);
    fs::path index_file_path = ".\\DataSet\\index.txt";
    if (fs::exists(index_file_path)) {
        fs::remove(index_file_path);
    }
    std::fstream index_file(index_file_path,
        std::ios::out | std::ios::app);

    std::vector<ImageRequest> request{ 
        ImageRequest("0", ImageType::DisparityNormalized, true) };
    int index = 0;
    
    while (is_busy_) {
        const std::vector<ImageResponse> &response =
            client_.simGetImages(request); 
		if (save_as_file) {
			std::string file_pfm_path{ ".\\DataSet\\Disparity\\" +
			std::to_string(index) + ".pfm" };
			if (fs::exists(file_pfm_path)) {
				fs::remove(file_pfm_path);
			}
			std::fstream file_pfm{ file_pfm_path,
				std::ios::out | std::ios::binary };
			int scalef = Utils::isLittleEndian() ? -1 : 1;
			int width = response[0].width;
			int height = response[0].height;
			file_pfm << "Pf\n" << width << " " << height << "\n"
				<< scalef << "\n";
			float fvalue;
			for (int i = 0; i < height; ++i) {
				for (int j = 0; j < width; ++j) {
					fvalue = response[0].image_data_float.data()[i];
					file_pfm.write(reinterpret_cast<char*>(&fvalue), sizeof(fvalue));
				}
			}
			file_pfm.close();
			std::cout << "record " << index << std::endl;
			++index;
		}
		disparity_retreived_.push(response[0]);
		// temporary test
		std::this_thread::sleep_for(
			std::chrono::milliseconds(500));
    }
}

void ImageRecordComponent::Behave() {
    is_busy_ = true;
    /*thread_handle_ = std::thread{ 
        &ImageRecordComponent::Record, this };*/
    //thread_handle_.detach();
}
