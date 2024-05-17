
/*
* File name:        robot.hpp
* Author:           Noam Hasson
* Description:      
*/
#pragma once

//=====================================================================================================================
// DECLARATIONS
//=====================================================================================================================

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "parameters.hpp"

//=====================================================================================================================
// TYPE DEFINITION
//=====================================================================================================================
class Robot {
    public:

    class Ekf {
        Eigen::Vector3f state;
        Eigen::Matrix3d covMatrix;
        Eigen::Matrix3d processNoise;
        Eigen::Matrix3d measurementNoise;
        void init(Eigen::Vector3f initialState,
                  float initalCovariancesXY, 
                  float initalCovariancesTheta, 
                  float processNoiseXY,
                  float processNoiseTheta,
                  float measurementNoiseXY,
                  float measurementNoiseTheta);
        void prediction(Eigen::Vector2f u);
    };

    public:
    Eigen::Vector3f stateGT;
    Robot(Eigen::Vector3f initialState)
        : stateGT(initialState) {}
    void step(Eigen::Vector2f u);
    };
