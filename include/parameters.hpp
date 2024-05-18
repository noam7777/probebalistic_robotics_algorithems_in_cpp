
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
#define WORLD_WIDTH 100
#define WORLD_HEIGHT 100

// ekf:
// measurement noises:
#define EKF_COMPASS_VARIANCE 2.0f
#define EKF_GPS_XY_VARIANCE 5.0f // used in particle filter too, TODO: set new parameter for particle filter
#define EKF_INITIAL_STATE_UNCERTAINTY_XY 30.0f
#define EKF_INITIAL_STATE_UNCERTAINTY_THETA 3.3f


//particle filter:
#define PF_RANDOM_CONTROL_NOISE_LINEAR 2.0f
#define PF_RANDOM_CONTROL_NOISE_ANGULAR 1.0f
#define PF_INITIAL_STATE_UNCERTAINTY_XY 3.0f
#define PF_INITIAL_STATE_UNCERTAINTY_THETA 0.3f
