#ifndef DYNAMIC_DEGRADATION_FILTER_CHARLIE_HPP
#define DYNAMIC_DEGRADATION_FILTER_CHARLIE_HPP

#include <unordered_map>
#include <memory>
#include <string>
#include <deque>
#include <iostream> // for optimization status messages

#include <nlopt.hpp>

#include "CollectivePerception.hpp"
#include "SensorDegradationFilter.hpp"
#include "ELBO.hpp"

class DynamicDegradationFilterCharlie : public SensorDegradationFilter
{
public:
    DynamicDegradationFilterCharlie(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                                    double lower_bound = 0.5 + ZERO_APPROX,
                                    double upper_bound = 1.0 - ZERO_APPROX);

    virtual void Init() override;

    virtual void Estimate();

private:
    virtual void Predict();

    virtual void Update();

    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

    std::shared_ptr<DistributionParameters> distribution_params_ptr_;

    nlopt::opt ELBO_optimizer_;

    nlopt::opt MAP_optimizer_;

    nlopt::result ELBO_optimization_status_;

    nlopt::result MAP_optimization_status_;

    ELBO elbo_;

    std::pair<std::vector<double>, double> MAP_outcome_;

    std::pair<std::vector<double>, double> ELBO_outcome_;

    std::deque<double> observations_;

    double model_a_;

    double variance_r_;
};

#endif