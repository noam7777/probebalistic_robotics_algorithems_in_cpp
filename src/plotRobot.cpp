#include "world.hpp"


int main()
{
    // Eigen::Vector2f robotPos;

    Robot rob1 = Robot(4.0f, 4.0f, 0.0f);
    World world;
    world.addRobotToWorld(rob1);

    for (int i = 0; i<5 ;i++) {
        ControlSignal u = ControlSignal(1.0f, 0.0f/* M_PI_4 */);
        rob1.step(u);
        world.addRobotToWorld(rob1);
    }
    world.plotWorld();
    return 0;
}