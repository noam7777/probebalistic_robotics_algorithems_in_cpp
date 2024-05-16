
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

//=====================================================================================================================
// TYPE DEFINITION
//=====================================================================================================================
struct ControlSignal
{
    float linearVel;
    float angularVel;
    ControlSignal(float linearVel_, float angularVel_) : linearVel(linearVel_), angularVel(angularVel_){}
};

class Robot {
    
    public:
    float x;
    float y;
    float theta;
    Robot(float initial_x, float initial_y, float initial_theta)
        : x(initial_x), y(initial_y), theta(initial_theta) {}
    void step(ControlSignal u);
    };
