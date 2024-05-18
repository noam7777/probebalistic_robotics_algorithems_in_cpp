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
    this->stateGT(2) += (u(1) * (DT_SEC + 0.2f));
    this->stateGT(0) += (u(0) * DT_SEC * cosf(stateGT(2)));
    this->stateGT(1) += (u(0) * DT_SEC * sinf(stateGT(2)));
}

void Robot::Ekf::prediction(Eigen::Vector2f u) {
    /* u = [v_cmd, w_cmd]^T */
    this->state(2) += (u(1) * DT_SEC);
    this->state(0) += (u(0) * DT_SEC * cosf(state(2)));
    this->state(1) += (u(0) * DT_SEC * sinf(state(2)));

    Eigen::Matrix3f jacobianG;
    jacobianG << 1, 0, -sin(state(2)),
                 0, 1, cos(state(2)), 
                 0, 0, 1;

    this->covMatrix = jacobianG * this->covMatrix * jacobianG.transpose() + this->processNoise;
}

Eigen::Vector3f Robot::expectedMeasurement(Eigen::Vector3f state) {
    // in more complex measurement model, this will be more complex;
    return state;
}
void Robot::Ekf::update(Eigen::Vector3f z) {
    Eigen::Matrix3f measurementJacobian = Eigen::Matrix<float, 3, 3>::Identity();
    Eigen::Matrix3f kalmanGain = covMatrix * measurementJacobian.transpose() * (measurementJacobian * covMatrix * measurementJacobian.transpose() + measurementNoise).inverse();
    this->state += kalmanGain*(z - this->state/* replace with expectedMeasurement */);
    this->covMatrix = (Eigen::Matrix<float, 3, 3>::Identity() - kalmanGain * measurementJacobian) * this->covMatrix;
}

void Robot::Ekf::init(Eigen::Vector3f initialState) {
    float initalCovariancesXY = 1.0f;
    float initalCovariancesTheta = 1.0f;
    float processNoiseXY = 1.0f;
    float processNoiseTheta = 1.0f;


    this->state = initialState;
    this->covMatrix << EKF_INITIAL_STATE_UNCERTAINTY_XY, 0.0f, 0.0f,
                        0.0f, EKF_INITIAL_STATE_UNCERTAINTY_XY, 0.0f,
                        0.0f, 0.0f, EKF_INITIAL_STATE_UNCERTAINTY_THETA;
    this->processNoise << processNoiseXY, 0.0f, 0.0f, 
                            0.0f, processNoiseXY, 0.0f,
                            0.0f, 0.0f, processNoiseTheta;
    this->measurementNoise << EKF_GPS_XY_VARIANCE, 0.0f, 0.0f,
                                0.0f, EKF_GPS_XY_VARIANCE, 0.0f,
                                0.0f, 0.0f, EKF_COMPASS_VARIANCE;
}
