#pragma once
#include <thread>
#include <stdexcept>
#include <experimental/filesystem>
#include <chrono>

namespace fs = std::experimental::filesystem;

// bugged
class ThreadRaii {
    std::thread thread_;
public:
    explicit ThreadRaii(std::thread thread)
        : thread_(std::move(thread)) {
        if (!thread.joinable()) {
            throw std::logic_error("No thread!");
        }
    }
    ~ThreadRaii() {
        thread_.detach();
    }
    ThreadRaii(ThreadRaii const&) = delete;
    ThreadRaii& operator=(ThreadRaii const&) = delete;
};

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
                std::chrono::duration_cast<milliseconds>(diff).count());
        }
        return 0;
    };
private:
    std::chrono::system_clock::time_point start_;
}
