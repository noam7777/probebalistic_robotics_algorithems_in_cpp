
/*
* File name:        particle_filter.hpp
* Author:           Noam Hasson
* Description:      
*/
#pragma once

//=====================================================================================================================
// DECLARATIONS
//=====================================================================================================================

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "parameters.hpp"

//=====================================================================================================================
// TYPE DEFINITION
//=====================================================================================================================
class Particle {
    public:
    Eigen::Vector3f state;
    float weight;
};

class ParticleFilter {
    public:
    std::vector<Particle> particles;
    int particlesCount;
    float sumOfWeights;
    void init(Eigen::Vector3f initialState, int particleCount, float posVariance, float angleVariance);
    void predictionSampleAndUpdateWeights(Eigen::Vector2f u, Eigen::Vector3f gpsCompassMeasurement);
    void lowVarianceSampler(void);
};
