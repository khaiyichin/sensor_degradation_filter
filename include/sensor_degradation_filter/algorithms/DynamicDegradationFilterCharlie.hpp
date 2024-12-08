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

    virtual void Reset() override
    {
        SensorDegradationFilter::Reset();

        MAP_outcome_.first[0] = initial_mean_;
        ELBO_outcome_.first[0] = initial_cov_;

        informed_estimate_history_.clear();
    }

    virtual void Estimate();

private:
    virtual void Predict();

    virtual void Update();

    virtual void ComputeWeightedAverageFillRatio();

    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

    std::shared_ptr<DistributionParameters> distribution_params_ptr_;

    nlopt::opt ELBO_optimizer_;

    nlopt::opt MAP_optimizer_;

    nlopt::result ELBO_optimization_status_;

    nlopt::result MAP_optimization_status_;

    ELBO elbo_;

    std::pair<std::vector<double>, double> MAP_outcome_ = {{-1.0}, -1.0};

    std::pair<std::vector<double>, double> ELBO_outcome_ = {{-1.0}, -1.0};

    std::deque<double> informed_estimate_history_;

    size_t max_informed_estimate_history_length_ = 0;

    double model_b_ = 0.0; // our model of what the drift coefficient is (default: no drift)

    double variance_r_ = -1.0; // our model of what the diffusion coefficient is (default: invalid value; requires setting)

    double initial_mean_ = -1.0; // initial guess for the MAP estimate (default: invalid value; requires setting)

    double initial_cov_ = -1.0; // initial guess for the ELBO estimate (default: invalid value: requires setting)
};

#endif