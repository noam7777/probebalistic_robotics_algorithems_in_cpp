
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
#include "rssi_model.hpp"

//=====================================================================================================================
// TYPE DEFINITION
//=====================================================================================================================
class Particle {
    public:
    Eigen::Vector3f state;
#ifdef PF_SIM_USE_RSSI_MEASUREMENTS
    RssiModel rssiModel;
#endif
    float weight;

    Particle(const Eigen::Vector3f& state, const RssiModel& rssiModel)
        : state(state), rssiModel(rssiModel) {
        this->weight = 0.0f;
    }
    float likelyhoodToGetRssiMeasurementFromLandmark(float rssiMeasurement);
};

class ParticleFilter {
    public:
    std::vector<Particle> particles;
    int particlesCount;
    float sumOfWeights;
    void init(Eigen::Vector3f initialState, int particleCount, float posVariance, float angleVariance);
#ifdef PF_SIM_USE_GPS_AND_COMPASS_MEASUREMENTS
    void predictionSampleAndUpdateWeights(Eigen::Vector2f u, Eigen::Vector3f gpsCompassMeasurement);
#endif 
#ifdef PF_SIM_USE_DIRECT_RANGE_MEASUREMENTS
    void predictionSampleAndUpdateWeights(Eigen::Vector2f u, float rangeFromLandmark, float orientationGT);
#endif
#ifdef PF_SIM_USE_RSSI_MEASUREMENTS
    void predictionSampleAndUpdateWeightsWithRssiMeasurement(Eigen::Vector2f u, float rssi, float orientationGT);
#endif
    void lowVarianceSampler(void);
    void uniformRadialSampler(float Range);
};
