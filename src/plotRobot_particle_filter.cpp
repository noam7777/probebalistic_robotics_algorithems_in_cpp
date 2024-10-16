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
    Eigen::Vector3f initialStateGT, initialStateEstimationMean;
    initialStateGT << ROBOT_INTIAL_STATE_X, ROBOT_INTIAL_STATE_Y, ROBOT_INTIAL_STATE_THETA;
    initialStateEstimationMean << PF_INITIAL_STATE_X, PF_INITIAL_STATE_Y, PF_INITIAL_STATE_THETA;

    Robot rob1 = Robot(initialStateGT);
    int particleCount = 100;

    rob1.pf.init(initialStateEstimationMean, PF_PARTICLE_COUNT, PF_INITIAL_STATE_UNCERTAINTY_XY, PF_INITIAL_STATE_UNCERTAINTY_THETA);
    world.addRobotToArchive(rob1);

    for (int i = 0; i<WORLD_TOTAL_TIME_STEPS ;i++) {
        Eigen::Vector2f u;
        // u << 6.0f, (0.15f - (0.01 * i));
        u << 6.0f, 0.2 * (sin(0.15f * (float)i)) + 0.1 * (sin(0.03f * (float)i));
        rob1.step(u);
        // u << 6.0f, 0.0f;
        // use gps and compass to locelize the robot
        // Eigen::Vector3f gpsCompassMeasurement = world.getGpsCompassMeasurement(rob1);
        // rob1.pf.predictionSampleAndUpdateWeights(u,  gpsCompassMeasurement);
        // rob1.pf.predictionSampleAndUpdateWeights(u,  gpsCompassMeasurement);

        // use range from landmark to locelize the robot
        float rangeFromLandmarkMeasurement = world.getRangeFromLandmarkMeasurement(rob1);
        rob1.pf.predictionSampleAndUpdateWeights(u,  rangeFromLandmarkMeasurement, rob1.stateGT[2]);
        rob1.pf.lowVarianceSampler();
        if (i % 20 == 0) {
            rob1.pf.uniformRadialSampler(world.getRangeFromLandmarkMeasurement(rob1));
        }
        world.addRobotToArchive(rob1);
    }

    // world.plotWorld(true, false, true, true);
    world.animateRobotStates(true, false, true, true);
    return 0;
}