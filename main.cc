#include "world.h"
#include "Drone.h"

int main() 
{
    World world = World::Instance();
    world.Loop();

    return 0;
}
