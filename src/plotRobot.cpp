#include "world.hpp"


int main()
{
    // Eigen::Vector2f robotPos;
    World world;
    Eigen::Vector3f initialState;
    initialState << 4.0f, 4.0f, 0.0f;
    Robot rob1 = Robot(initialState);
    
    world.addRobotToWorld(rob1);

    for (int i = 0; i<5 ;i++) {
        Eigen::Vector2f u;
        u << 1.0f, 0.0f;
        rob1.step(u);
        world.addRobotToWorld(rob1);
    }
    world.plotWorld();
    return 0;
}