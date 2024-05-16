#include "world.hpp"

int main()
{
    // Eigen::Vector2f robotPos;

    Robot rob1 = Robot(3.0f, 3.0f, 0.1f);
    Robot rob2 = Robot(4.0f, 6.0f, 0.4f);
    World world;
    world.addRobotToWorld(rob1);
    world.addRobotToWorld(rob2);
    world.plotWorld();
    return 0;
}