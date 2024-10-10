/*
* File name:        utils.hpp
* Author:           Noam Hasson
* Description:      
*/

//=====================================================================================================================
// DECLARATIONS
//=====================================================================================================================
#include "particle_filter.hpp"
#include <iostream>
#include "utils.hpp"
//=====================================================================================================================
// GLOBALS
//=====================================================================================================================


//=====================================================================================================================
// FUNCTIONS
//=====================================================================================================================

void ParticleFilter::init(Eigen::Vector3f initialState, int particleCount, float posVariance, float angleVariance) {
    this->particlesCount = particleCount;
    for (int i = 0; i< this->particlesCount; i++)
    {
        Eigen::Vector3f randomParticleState;
        Particle randParticle;
        randParticle.state << generateGaussianRandom(initialState(0), posVariance),
                              generateGaussianRandom(initialState(1), posVariance),
                              generateGaussianRandom(initialState(2), angleVariance);
        this->particles.push_back(randParticle);
    }
    return;
}

// void ParticleFilter::predictionSampleAndUpdateWeights(Eigen::Vector2f u, Eigen::Vector3f gpsCompassMeasurement) {
//     this->sumOfWeights = 0;
//     for (auto& particle : this->particles) {
//         particle.state(2) += (u(1) * DT_SEC);
//         particle.state(0) += (u(0) * DT_SEC * cosf(particle.state(2)));
//         particle.state(1) += (u(0) * DT_SEC * sinf(particle.state(2)));
//         particle.weight = likelyhoodToGetMeasurementGpsCompassFromState(gpsCompassMeasurement, particle.state);
//         this->sumOfWeights += particle.weight;
//     }
// }


void ParticleFilter::predictionSampleAndUpdateWeights(Eigen::Vector2f u, Eigen::Vector3f gpsCompassMeasurement) { // with random noise at the control input
    this->sumOfWeights = 0;
    for (auto& particle : this->particles) {
            particle.state(2) += ((u(1) + (getRandomNumber()-0.5f) * 2.0f * PF_RANDOM_CONTROL_NOISE_ANGULAR) * DT_SEC);
            particle.state(0) += ((u(0) + (getRandomNumber()-0.5f) * 2.0f * PF_RANDOM_CONTROL_NOISE_LINEAR) * DT_SEC * cosf(particle.state(2)));
            particle.state(1) += ((u(0) + (getRandomNumber()-0.5f) * 2.0f * PF_RANDOM_CONTROL_NOISE_LINEAR) * DT_SEC * sinf(particle.state(2)));
        particle.weight = likelyhoodToGetMeasurementGpsCompassFromState(gpsCompassMeasurement, particle.state);
        this->sumOfWeights += particle.weight;
    }
}

void ParticleFilter::predictionSampleAndUpdateWeights(Eigen::Vector2f u, float rangeFromLandmark, float orientationGT) { // using range from Landmark as measurement
    this->sumOfWeights = 0;
    for (auto& particle : this->particles) {
        particle.state(2) = orientationGT;
        particle.state(0) += ((u(0) + (getRandomNumber()-0.5f) * 2.0f * PF_RANDOM_CONTROL_NOISE_LINEAR) * DT_SEC * cosf(particle.state(2))) + ((getRandomNumber()-0.5f) * 2 * PF_RANDOM_POSITION_NOISE * DT_SEC);
        particle.state(1) += ((u(0) + (getRandomNumber()-0.5f) * 2.0f * PF_RANDOM_CONTROL_NOISE_LINEAR) * DT_SEC * sinf(particle.state(2))) + ((getRandomNumber()-0.5f) * 2 * PF_RANDOM_POSITION_NOISE * DT_SEC);
        particle.weight = likelyhoodToGetMeasurementRangeFromLandmark(rangeFromLandmark, particle.state);
        this->sumOfWeights += particle.weight;
    }
}


void ParticleFilter::lowVarianceSampler(void) {
    if (this->sumOfWeights != 0.0f) {
        int newParticleCount = 0;
        std::vector<Particle> newParticleSamples;
        float particleSelectorInterval = ((this->sumOfWeights) / (this->particlesCount));
        float particleSelector = (getRandomNumber() * particleSelectorInterval);
        float runningSumOfWeights = 0.0f;

        for (auto& particle : this->particles) {
            runningSumOfWeights += particle.weight;
            while (particleSelector <= runningSumOfWeights) {
                particleSelector += particleSelectorInterval;
                newParticleSamples.push_back(particle);
                newParticleCount ++;
            }
        }
        this->particles = newParticleSamples;
    }
}
