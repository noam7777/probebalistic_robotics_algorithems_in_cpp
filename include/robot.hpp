
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
#include "particle_filter.hpp"
#include "rssi_model.hpp"

//=====================================================================================================================
// TYPE DEFINITION
//=====================================================================================================================
class Robot {
    public:

    class Ekf {
        public:
        Eigen::Vector3f state;
        Eigen::Matrix3f covMatrix;
        Eigen::Matrix3f processNoise;
        Eigen::Matrix3f measurementNoise;
        void init(Eigen::Vector3f initialState);
        void prediction(Eigen::Vector2f u);
        void update(Eigen::Vector3f z);
    };

    public:
    Eigen::Vector3f expectedMeasurement(Eigen::Vector3f state);
    Ekf ekf;
    ParticleFilter pf;
    RssiModel rssiModelGT;
    Eigen::Vector3f stateGT;                // the ground truth location and orientation of the robot
    Robot(Eigen::Vector3f initialState, RssiModel rssiModel)
        : stateGT(initialState), rssiModelGT(rssiModel) {}
    void step(Eigen::Vector2f u);
};
