
/*
* File name:        world.hpp
* Author:           Noam Hasson
* Description:      
*/
#pragma once

//=====================================================================================================================
// DECLARATIONS
//=====================================================================================================================

#define DT_SEC 1.0
#define WORLD_WIDTH 1100
#define WORLD_HEIGHT 1000
#define WORLD_TOTAL_TIME_STEPS 400

#define LANDMARK_LOCATION_X 400.0f
#define LANDMARK_LOCATION_Y 350.0f
// #define LANDMARK_RANGE_ERROR_BIAS -30.0f
// #define LANDMARK_RANGE_ERROR_SCALE 0.8f
#define LANDMARK_RANGE_ERROR_BIAS 30.0f
#define LANDMARK_RANGE_ERROR_SCALE 0.8f

#define ROBOT_INTIAL_STATE_X 150.0f
#define ROBOT_INTIAL_STATE_Y 110.0f
#define ROBOT_INTIAL_STATE_THETA 4.5f

// ekf:
// measurement noises:
#define EKF_COMPASS_VARIANCE 2.0f
#define EKF_GPS_XY_VARIANCE 5.0f // used in particle filter too, TODO: set new parameter for particle filter
#define EKF_INITIAL_STATE_UNCERTAINTY_XY 30.0f
#define EKF_INITIAL_STATE_UNCERTAINTY_THETA 3.3f


//particle filter:
#define PF_PARTICLE_COUNT                       30
#define PF_RANDOM_CONTROL_NOISE_LINEAR          2.0f
#define PF_RANDOM_CONTROL_NOISE_ANGULAR         1.0f
#define PF_RANDOM_POSITION_NOISE                10.0f

// particle filter gps - suing pos and angle measurements
#define PF_INITIAL_STATE_UNCERTAINTY_XY         3000.0f
#define PF_INITIAL_STATE_UNCERTAINTY_THETA      0.3f

//particle filter rssi - using range from landmark measurements 
#define PF_UNIFORM_RADIAL_SAMPLER_SAMPLER_COUNT 8 // must be smaller then PF_PARTICLE_COUNT
#define PF_UNIFORM_RADIAL_SAMPLER_TIME_INTERVAL 40 // time interval between each uniform radial sample
#define PF_INITIAL_STATE_X                      300.0f
#define PF_INITIAL_STATE_Y                      300.0f
#define PF_INITIAL_STATE_THETA                  4.5f
#define PF_RANGE_FROM_LANDMARK_UNCERTAINTY 30.3f
// #define PF_SIM_USE_GPS_AND_COMPASS_MEASUREMENTS
// #define PF_SIM_USE_DIRECT_RANGE_MEASUREMENTS
#define PF_SIM_USE_RSSI_MEASUREMENTS

#define RSSI_D0 10 // reference range
#define RSSI_PL 1.0f  // path loss at reference range
#define RSSI_PT 32.0f // power of transmission
#define RSSI_N  2.0f  // transmission loss exponent

#define RSSI_DEFAULT_PL 1.0f // initial guess for the path loss at reference range
#define RSSI_DEFAULT_N 2.0f // initial guess for the transmission loss exponent