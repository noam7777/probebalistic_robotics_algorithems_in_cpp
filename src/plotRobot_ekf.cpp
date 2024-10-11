#include "world.hpp"


int main()
{
    // Eigen::Vector2f robotPos;
    World world;
    Eigen::Vector3f initialState;
    initialState << 50.0f, 10.0f, 0.0f;
    Robot rob1 = Robot(initialState);
    
    rob1.ekf.init(initialState);
    world.addRobotToArchive(rob1);

    for (int i = 0; i< WORLD_TOTAL_TIME_STEPS;i++) {
        Eigen::Vector2f u;
        u << 3.0f, 0.2f;
        rob1.step(u);
        u << 4.0f, 0.0f;
        rob1.ekf.prediction(u);
        Eigen::Vector3f measurement = world.getGpsCompassMeasurement(rob1);
        rob1.ekf.update(measurement);
        world.addRobotToArchive(rob1);
    }
    world.plotWorld(true, true, false, false);
    return 0;
}