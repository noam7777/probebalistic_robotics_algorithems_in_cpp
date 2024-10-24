
//=====================================================================================================================
// TYPE DEFINITION
//=====================================================================================================================

//=====================================================================================================================
// GLOBALS
//=====================================================================================================================

//=====================================================================================================================
// FUNCTIONS
//=====================================================================================================================
float getRandomNumber(void);
float generateGaussianRandom(float mean, float variance);
float multivariateGaussian(const Eigen::VectorXf x, const Eigen::VectorXf mean, const Eigen::MatrixXf cov);
float likelyhoodToGetMeasurementGpsCompassFromState(Eigen::Vector3f gpsCompassMeasurement, Eigen::Vector3f state);
float likelyhoodToGetMeasurementRangeFromLandmark(float rangeFromLandmark, Eigen::Vector3f state);
float gaussian1D(float x, float mean, float variance);
