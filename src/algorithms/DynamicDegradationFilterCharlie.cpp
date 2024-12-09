#include "algorithms/DynamicDegradationFilterCharlie.hpp"

DynamicDegradationFilterCharlie::DynamicDegradationFilterCharlie(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                                                                 double lower_bound /* 0.5 + ZERO_APPROX */,
                                                                 double upper_bound /* 1.0 - ZERO_APPROX */)
    : SensorDegradationFilter(),
      MAP_optimizer_(nlopt::LN_NELDERMEAD, 1),
      ELBO_optimizer_(nlopt::LN_NELDERMEAD, 1),
      elbo_(lower_bound, upper_bound, SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR)
{
    collective_perception_algo_ptr_ = col_per_ptr;
}

void DynamicDegradationFilterCharlie::Init()
{
    // Get the pointer to the distribution parameters (related to the distribution, not the filter parameters)
    distribution_params_ptr_ = elbo_.GetDistributionParametersPtr();

    /* Note that distribution_params_ptr_->SensorAccuracyInternalFactor is set by ELBO already */

    std::pair<double, double> limits = elbo_.GetIntegrationLimits();
    distribution_params_ptr_->PredictionLowerBound = limits.first;
    distribution_params_ptr_->PredictionUpperBound = limits.second;
    distribution_params_ptr_->SurrogateLowerBound = limits.first;
    distribution_params_ptr_->SurrogateUpperBound = limits.second;

    // Define lambda function for the MAP object function
    auto nlopt_wrapped_MAP_fcn = [](const std::vector<double> &x,
                                    std::vector<double> &grad,
                                    void *my_func_data) -> double
    {
        // Cast into SensorAccuracyELBO type object
        SensorAccuracyELBO *elbo_obj_ptr = static_cast<SensorAccuracyELBO *>(my_func_data);

        elbo_obj_ptr->ComputePredictionNormalizationConstant();

        return joint_density_fcn(x[0], elbo_obj_ptr->GetDistributionParametersPtr().get());
    };

    // Define lambda function for the ELBO objective function
    auto nlopt_wrapped_ELBO_fcn = [](const std::vector<double> &x,
                                     std::vector<double> &grad,
                                     void *my_func_data) -> double
    {
        // Cast into SensorAccuracyELBO type object
        SensorAccuracyELBO *elbo_obj_ptr = static_cast<SensorAccuracyELBO *>(my_func_data);

        elbo_obj_ptr->GetDistributionParametersPtr()->SurrogateScale = x[0]; // update the candidate scale value

        elbo_obj_ptr->ComputeELBO();

        return elbo_obj_ptr->GetELBOResult();
    };

    // Define the objective functions
    MAP_optimizer_.set_max_objective(nlopt_wrapped_MAP_fcn, &elbo_);
    MAP_optimizer_.set_lower_bounds(limits.first);
    MAP_optimizer_.set_upper_bounds(limits.second);
    MAP_optimizer_.set_xtol_abs(1e-9);

    ELBO_optimizer_.set_max_objective(nlopt_wrapped_ELBO_fcn, &elbo_);
    ELBO_optimizer_.set_lower_bounds(3e-2 * distribution_params_ptr_->SensorAccuracyInternalFactor); // smallest std dev value (any smaller the ELBO evaluation will encounter nan/inf values)
    ELBO_optimizer_.set_upper_bounds(HUGE_VAL);                                                      // HUGE_VAL is basically infinity
    ELBO_optimizer_.set_xtol_abs(1e-9);

    // Initialize the prior (initial guess)
    initial_mean_ = std::stod(params_ptr_->FilterSpecificParams["init_mean_MAP"]) * distribution_params_ptr_->SensorAccuracyInternalFactor;
    initial_std_dev_ = std::stod(params_ptr_->FilterSpecificParams["init_std_dev_ELBO"]) * distribution_params_ptr_->SensorAccuracyInternalFactor;

    MAP_outcome_.first[0] = initial_mean_;
    ELBO_outcome_.first[0] = initial_std_dev_;

    // Extract the prediction model parameters
    model_b_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_model_B"].c_str()) * distribution_params_ptr_->SensorAccuracyInternalFactor;
    std_dev_r_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_std_dev_R"].c_str()) * distribution_params_ptr_->SensorAccuracyInternalFactor;

    // Set the maximum number of past informed estimates to collect
    max_informed_estimate_history_length_ = collective_perception_algo_ptr_->GetParamsPtr()->MaxObservationQueueSize;
}

void DynamicDegradationFilterCharlie::Reset()
{
    // Call parent Reset
    SensorDegradationFilter::Reset();

    // Clear MAP and ELBO optimizer values
    MAP_outcome_.first[0] = initial_mean_;
    ELBO_outcome_.first[0] = initial_std_dev_;

    // Clear buffer for informed estimate (if used)
    informed_estimate_history_.clear();

    // Reset ELBO and reinitialize the bounds for the internal distributions
    elbo_.Reset();

    std::pair<double, double> limits = elbo_.GetIntegrationLimits();
    distribution_params_ptr_->PredictionLowerBound = limits.first;
    distribution_params_ptr_->PredictionUpperBound = limits.second;
    distribution_params_ptr_->SurrogateLowerBound = limits.first;
    distribution_params_ptr_->SurrogateUpperBound = limits.second;
}

void DynamicDegradationFilterCharlie::Predict()
{
    // Compute predicted mean and std dev based on a = 1, b = model_b_, and r = std_dev_r_
    distribution_params_ptr_->PredictionMean = MAP_outcome_.first[0] + params_ptr_->FilterActivationPeriodTicks * model_b_;                                           // a^t*x + b*t
    distribution_params_ptr_->PredictionStdDev = std::sqrt(std::pow(ELBO_outcome_.first[0], 2) + std::pow(std_dev_r_, 2) * params_ptr_->FilterActivationPeriodTicks); // a^(2*t) * sigma^2 + r*t
}

void DynamicDegradationFilterCharlie::Update()
{
    // Update the binomial likelihood values
    distribution_params_ptr_->NumObservations = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations;
    distribution_params_ptr_->NumBlackTilesSeen = collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen;

    // Estimate the truncated normal mean
    MAP_optimization_status_ = MAP_optimizer_.optimize(MAP_outcome_.first, MAP_outcome_.second);

    // Check if optimization is successful
    if (MAP_optimization_status_ < 0)
    {
        std::cout << "MAP Optimization unsuccessful: flag=" << MAP_optimization_status_
                  << " with f(" << MAP_outcome_.first[0] << ") = " << MAP_outcome_.second
                  << std::endl;
    }

    // Estimate the truncated normal variance
    distribution_params_ptr_->SurrogateLoc = MAP_outcome_.first[0];

    ELBO_optimization_status_ = ELBO_optimizer_.optimize(ELBO_outcome_.first, ELBO_outcome_.second);

    // Check if optimization is successful
    if (ELBO_optimization_status_ < 0)
    {
        std::cout << "ELBO Optimization unsuccessful: flag=" << ELBO_optimization_status_
                  << " with f(" << ELBO_outcome_.first[0] << ") = " << ELBO_outcome_.second
                  << std::endl;
    }
}

void DynamicDegradationFilterCharlie::Estimate()
{
    // Collect informed estimate into weighted average queue (if an observation queue is used)
    if (max_informed_estimate_history_length_ > 1)
    {
        ComputeWeightedAverageFillRatio();
    }
    else
    {
        distribution_params_ptr_->FillRatio = collective_perception_algo_ptr_->GetInformedVals().X;
    }

    // Execute prediction step
    Predict();

    // Execute update step
    Update();

    // Update assumed sensor accuracy (should be converted OUT of internal units)
    params_ptr_->AssumedSensorAcc["b"] = MAP_outcome_.first[0] / distribution_params_ptr_->SensorAccuracyInternalFactor;
    params_ptr_->AssumedSensorAcc["w"] = MAP_outcome_.first[0] / distribution_params_ptr_->SensorAccuracyInternalFactor;
}

void DynamicDegradationFilterCharlie::ComputeWeightedAverageFillRatio()
{
    // Store the most recent informed estimate
    informed_estimate_history_.push_back(collective_perception_algo_ptr_->GetInformedVals().X);

    if (informed_estimate_history_.size() > max_informed_estimate_history_length_)
    {
        informed_estimate_history_.pop_front();
    }

    // Compute the weighted average (weighing older estimates as heavier)
    double numerator = 0.0;
    double denominator = informed_estimate_history_.size() * (informed_estimate_history_.size() + 1) / 2.0;

    for (size_t i = informed_estimate_history_.size(); i > 0; --i)
    {
        numerator += i * informed_estimate_history_[informed_estimate_history_.size() - i];
    }

    distribution_params_ptr_->FillRatio = numerator / denominator;
}