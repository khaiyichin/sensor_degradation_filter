#ifndef DYNAMIC_DEGRADATION_FILTER_CHARLIE_HPP
#define DYNAMIC_DEGRADATION_FILTER_CHARLIE_HPP

#include <unordered_map>
#include <memory>
#include <string>
#include <deque>
#include <iostream> // for optimization status messages

#include <nlopt.hpp>

#include "CollectivePerception.hpp"
#include "ExtendedKalmanFilter.hpp"
#include "ELBO.hpp"

#define SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_CHARLIE 100.0 // the sensor accuracy is magnified internally to prevent numerical errors in integration

class DynamicDegradationFilterCharlie : public ExtendedKalmanFilter
{
public:
    DynamicDegradationFilterCharlie(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                                    double lower_bound = 0.5 + ZERO_APPROX,
                                    double upper_bound = 1.0 - ZERO_APPROX);

    virtual void Init() override;

    virtual void Reset() override;

    virtual void Estimate();

private:
    void Predict();

    void Update();

    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

    std::shared_ptr<SensorAccuracyDistributionParameters> distribution_params_ptr_;

    nlopt::opt ELBO_optimizer_;

    nlopt::opt MAP_optimizer_;

    nlopt::result ELBO_optimization_status_;

    nlopt::result MAP_optimization_status_;

    SensorAccuracyELBO elbo_;

    std::pair<std::vector<double>, double> MAP_outcome_ = {{-1.0}, -1.0};

    std::pair<std::vector<double>, double> ELBO_outcome_ = {{-1.0}, -1.0};

    double prev_assumed_acc_ = -1.0;

    const std::vector<double> empty_vec_double_ = {}; // dummy variable; used to pass into the predict function which doesn't require additional arguments
};

#endif