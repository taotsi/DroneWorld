#pragma once
#include <thread>
#include <stdexcept>

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