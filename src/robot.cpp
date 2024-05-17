/*
* File name:        ex_points_extraction.h
* Author:           Noam Hasson
* Description:      This file handle the points extraction functionality of the drone, which enable the operator to send 
*                   a location that is observed by the drone for external use. 
*/

//=====================================================================================================================
// DECLARATIONS
//=====================================================================================================================
#include "parameters.hpp"
#include "robot.hpp"

void Robot::step(Eigen::Vector2f u) {
    this->stateGT(2) += (u(1) * DT_SEC);
    this->stateGT(0) += (u(0) * DT_SEC * cosf(stateGT(2)));
    this->stateGT(1) += (u(0) * DT_SEC * sinf(stateGT(2)));
}

void Robot::Ekf::prediction(Eigen::Vector2f u) {
    /* u = [v_cmd, w_cmd]^T */
    this->state(2) += (u(1) * DT_SEC);
    this->state(0) += (u(0) * DT_SEC * cosf(state(2)));
    this->state(1) += (u(0) * DT_SEC * sinf(state(2)));
}

void Robot::Ekf::init(Eigen::Vector3f initialState,
                      float initalCovariancesXY, 
                      float initalCovariancesTheta, 
                      float processNoiseXY,
                      float processNoiseTheta,
                      float measurementNoiseXY,
                      float measurementNoiseTheta) {
    this->state = initialState;
    this->covMatrix << initalCovariancesXY, 0, 0,
                        0, initalCovariancesXY, 0,
                        0, 0, initalCovariancesTheta;
    this->processNoise << processNoiseXY, 0, 0, 
                            0, processNoiseXY, 0,
                            0, 0, processNoiseTheta;
    this->measurementNoise << measurementNoiseXY, 0, 0,
                                0, measurementNoiseXY, 0,
                                0, 0, measurementNoiseTheta;
}