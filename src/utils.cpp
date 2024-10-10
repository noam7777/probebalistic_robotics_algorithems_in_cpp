/*
* File name:        utils.hpp
* Author:           Noam Hasson
* Description:      
*/

//=====================================================================================================================
// DECLARATIONS
//=====================================================================================================================

#include <random>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "parameters.hpp"
//=====================================================================================================================
// GLOBALS
//=====================================================================================================================


//=====================================================================================================================
// FUNCTIONS
//=====================================================================================================================

float getRandomNumber(void) {
    //return a random number from 0.0f to 1.0f
    // Create a random number generator and seed it with a random device
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // Define the range for the random numbers
    static std::uniform_real_distribution<> dis(0.0, 1.0);

    // Generate a random number
    float random_number = dis(gen);
    return random_number;
}

float generateGaussianRandom(float mean, float variance) {
    // Create a random number generator
    std::random_device rd;  // Seed generator
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine

    // Standard deviation is the square root of the variance
    float stddev = std::sqrt(variance);

    // Create a normal (Gaussian) distribution object
    std::normal_distribution<> d(mean, stddev);

    // Generate and return a random number
    return d(gen);
}

// Function to calculate the multivariate Gaussian value
float multivariateGaussian(const Eigen::VectorXf x, const Eigen::VectorXf mean, const Eigen::MatrixXf cov) {
    static const float PI = 3.14159265358979323846;
    float k = x.size();
    
    // Calculate the determinant of the covariance matrix
    float det = cov.determinant();
    
    // Calculate the inverse of the covariance matrix
    Eigen::MatrixXf invCov = cov.inverse();
    
    // Calculate the normalization factor
    float normFactor = 1.0 / std::pow(2 * PI, k / 2.0) / std::sqrt(det);
    
    // Calculate the exponent term
    Eigen::VectorXf diff = x - mean;
    float exponent = -0.5 * diff.transpose() * invCov * diff;
    
    // Calculate the final Gaussian value
    return normFactor * std::exp(exponent);
}

float gaussian1D(float x, float mean, float variance) {
    static const float PI = 3.14159265358979323846;
    
    // Calculate the normalization factor
    float normFactor = 1.0 / std::sqrt(2 * PI * variance);
    
    // Calculate the exponent term
    float diff = x - mean;
    float exponent = -0.5 * (diff * diff) / variance;
    
    // Calculate the final Gaussian value
    return normFactor * std::exp(exponent);
}

float likelyhoodToGetMeasurementGpsCompassFromState(Eigen::Vector3f gpsCompassMeasurement, Eigen::Vector3f state) {
    Eigen::Matrix3f gpsCompassCov;
    gpsCompassCov << EKF_GPS_XY_VARIANCE, 0.0f, 0.0f,
                     0.0f, EKF_GPS_XY_VARIANCE, 0.0f,
                     0.0f, 0.0f, EKF_COMPASS_VARIANCE;

    return multivariateGaussian(state, gpsCompassMeasurement, gpsCompassCov);
}

float likelyhoodToGetMeasurementRangeFromLandmark(float rangeFromLandmarkMeasurement, Eigen::Vector3f state) {
    Eigen::Vector2f landmarkLocation;
    Eigen::Vector2f robotLocation;
    landmarkLocation << LANDMARK_LOCATION_X, LANDMARK_LOCATION_Y;
    robotLocation << state[0] , state[1];
    float predictedRangeFromLandmark = (landmarkLocation - robotLocation).norm();
    return gaussian1D(rangeFromLandmarkMeasurement ,predictedRangeFromLandmark, PARTICLE_FILTER_RANGE_FROM_LANDMARK_UNCERTAINTY);
}
