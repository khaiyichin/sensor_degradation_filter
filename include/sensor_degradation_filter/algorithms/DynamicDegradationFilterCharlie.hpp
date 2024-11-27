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

    std::pair<std::vector<double>, double> MAP_outcome_;

    std::pair<std::vector<double>, double> ELBO_outcome_;

    std::deque<double> informed_estimate_history_;

    size_t max_informed_estimate_history_length_;

    double model_b_; // our model of what the drift coefficient is

    double variance_r_; // our model of what the diffusion coefficient is

    double initial_mean_;

    double initial_cov_;
};

#endif