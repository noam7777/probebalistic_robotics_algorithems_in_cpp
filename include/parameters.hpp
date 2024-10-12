
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
#define WORLD_WIDTH 750
#define WORLD_HEIGHT 750
#define WORLD_TOTAL_TIME_STEPS 200

#define LANDMARK_LOCATION_X 250.0f
#define LANDMARK_LOCATION_Y 150.0f

#define ROBOT_INTIAL_STATE_X 150.0f
#define ROBOT_INTIAL_STATE_Y 110.0f
#define ROBOT_INTIAL_STATE_THETA 4.5f

// ekf:
// measurement noises:
#define EKF_COMPASS_VARIANCE 2.0f
#define EKF_GPS_XY_VARIANCE 5.0f // used in particle filter too, TODO: set new parameter for particle filter
#define EKF_INITIAL_STATE_UNCERTAINTY_XY 30.0f
#define EKF_INITIAL_STATE_UNCERTAINTY_THETA 3.3f
#define PARTICLE_FILTER_RANGE_FROM_LANDMARK_UNCERTAINTY 3.3f


//particle filter:
#define PF_PARTICLE_COUNT                       30
#define PF_UNIFORM_RADIAL_SAMPLER_SAMPLER_COUNT 8 // must be smaller then PF_PARTICLE_COUNT
#define PF_UNIFORM_RADIAL_SAMPLER_TIME_INTERVAL 40 // time interval between each uniform radial sample
#define PF_RANDOM_CONTROL_NOISE_LINEAR          2.0f
#define PF_RANDOM_CONTROL_NOISE_ANGULAR         1.0f
#define PF_RANDOM_POSITION_NOISE                10.0f
#define PF_INITIAL_STATE_UNCERTAINTY_XY         3000.0f
#define PF_INITIAL_STATE_UNCERTAINTY_THETA      0.3f
#define PF_INITIAL_STATE_X                      300.0f
#define PF_INITIAL_STATE_Y                      300.0f
#define PF_INITIAL_STATE_THETA                  4.5f
