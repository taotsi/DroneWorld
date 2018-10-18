#pragma once
#include "components/base_component.h"

class CompactPlaneComponent : public BaseComponent {
public:
    CompactPlaneComponent(
        std::queue<std::vector<std::vector<Pillar>>>* 
        pillar_cluster_queue);
    ~CompactPlaneComponent();
    void Begin();
    void Updata(double DeltaTime);
    /* data */
    std::queue<std::vector<std::vector<Pillar>>>* pillar_cluster_queue_;
private:
    /* methods */
    void RunCompact();
};