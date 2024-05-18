/*
* File name:        plotRobot_particle_filter.cpp
* Author:           Noam Hasson
* Description:      excersise for using particle filter with a simple robot
*/

//=====================================================================================================================
// DECLARATIONS
//=====================================================================================================================
#include <random>
#include "world.hpp"

//=====================================================================================================================
// GLOBALS
//=====================================================================================================================


//=====================================================================================================================
// FUNCTIONS
//=====================================================================================================================


int main()
{
    // Eigen::Vector2f robotPos;
    World world;
    Eigen::Vector3f initialState;
    initialState << 50.0f, 6.0f, 0.0f;
    Robot rob1 = Robot(initialState);
    int particleCount = 10;

    rob1.pf.init(initialState, particleCount, PF_INITIAL_STATE_UNCERTAINTY_XY, PF_INITIAL_STATE_UNCERTAINTY_THETA);
    world.addRobotToArchive(rob1);

    for (int i = 0; i<20 ;i++) {
        Eigen::Vector2f u;
        u << 6.0f, 0.2f;
        rob1.step(u);
        u << 6.0f, 0.0f;

        Eigen::Vector3f gpsCompassMeasurement = world.getMeasurement(rob1);
        rob1.pf.predictionSampleAndUpdateWeights(u,  gpsCompassMeasurement);
        rob1.pf.lowVarianceSampler();
        world.addRobotToArchive(rob1);
    }

    world.plotWorld(true, false, true);
    return 0;
}