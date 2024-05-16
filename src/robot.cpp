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

void Robot::step(ControlSignal u) {
    this->theta += u.angularVel * DT_SEC;
    this->x += std::cos(this->theta) * u.linearVel * DT_SEC;
    this->y += std::sin(this->theta) * u.linearVel * DT_SEC;
}
