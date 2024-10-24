/*
* File name:        world.hpp
* Author:           Noam Hasson
* Description:      
*/
#pragma once

//=====================================================================================================================
// DECLARATIONS
//=====================================================================================================================

#include <vector>
#include "commonMath.hpp"
#include "robot.hpp"

//=====================================================================================================================
// TYPE DEFINITION
//=====================================================================================================================

class World {
    private:
    int width;
    int height;
    std::vector<Robot> robots;
    Eigen::Vector2f lendMark;

    public:
    
    void plotWorld(bool plotGt, bool plotEkfEstimation, bool plotParticleFilterEstimation, bool plotLandmark);
    void animateRobotStates(bool plotGt, bool plotEkfEstimation, bool plotParticleFilterEstimation, bool plotLandmark);
    void addRobotToArchive(Robot robot);
    void cleanWorld(void);
    Eigen::Vector3f getGpsCompassMeasurement(Robot robot);
    float getRangeFromLandmarkMeasurement(Robot robot);
    float getRssiMeasurementFromLandmark(Robot robot);
    World(void) {
        width = WORLD_WIDTH;
        height = WORLD_HEIGHT;
        lendMark << LANDMARK_LOCATION_X, LANDMARK_LOCATION_Y;
    }
};

//=====================================================================================================================
// GLOBALS
//=====================================================================================================================


//=====================================================================================================================
// FUNCTIONS
//=====================================================================================================================
