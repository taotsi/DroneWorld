#pragma once

#include <thread>
#include <stdexcept>
#include <experimental/filesystem>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <chrono>

#include "nlohmann/json.hpp"

namespace droneworld{

using json = nlohmann::json;
namespace fs = std::experimental::filesystem;

class TimeBase{
public:
    TimeBase()
        : start_(std::chrono::system_clock::time_point::min()) {};
    void Clear() {
        start_ = std::chrono::system_clock::time_point::min();
    };
    bool IsStarted() const {
        return (start_.time_since_epoch() != 
            std::chrono::system_clock::duration(0));
    };
    void Start() {
        start_ = std::chrono::system_clock::now();
    };
    unsigned long GetMs() {
        if(IsStarted()){
            std::chrono::system_clock::duration diff;
            diff = std::chrono::system_clock::now() - start_;
            return static_cast<unsigned>(
                std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());
        }
        return 0;
    };
private:
    std::chrono::system_clock::time_point start_;
};

class SettingsJsonHandler{
public:
    SettingsJsonHandler() {
        std::system("reg query \"HKCU\\Software\\Microsoft\\Windows\\CurrentVersion\\Explorer\\Shell Folders\" /v Personal > temp.txt");
        std::ifstream file{"temp.txt"};
        std::string line;
        getline(file, line); getline(file, line); getline(file, line);
        file.close();
        fs::remove("temp.txt");
        std::stringstream ss{line};
        std::string path;
        ss >> path; ss >> path; ss >> path;
        json_path_ = path + "\\AirSim\\settings.json";
        std::ifstream json_file{json_path_};
        json_file >> json_data_;
    };
    ~SettingsJsonHandler() {};
    json& GetJsonData(){
        return json_data_;
    }
private:
    std::string json_path_;
    json json_data_;
};

} // namespace droneworld