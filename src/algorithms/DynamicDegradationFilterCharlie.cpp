#include "algorithms/DynamicDegradationFilterCharlie.hpp"

DynamicDegradationFilterCharlie::DynamicDegradationFilterCharlie(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                                                                 double lower_bound /* 0.5 + ZERO_APPROX */,
                                                                 double upper_bound /* 1.0 - ZERO_APPROX */)
    : SensorDegradationFilter(),
      MAP_optimizer_(nlopt::LN_NELDERMEAD, 1),
      ELBO_optimizer_(nlopt::LN_NELDERMEAD, 1),
      elbo_(lower_bound, upper_bound)
{
    collective_perception_algo_ptr_ = col_per_ptr;
}

void DynamicDegradationFilterCharlie::Init()
{
    // Set up usage of observation queue
    collective_perception_algo_ptr_->GetParamsPtr()->UseObservationQueue = true;
    collective_perception_algo_ptr_->GetParamsPtr()->MaxObservationQueueSize = std::stoi(params_ptr_->FilterSpecificParams["observation_queue_size"]);

    // Get the pointer to the distribution parameters (related to the distribution, not the filter parameters)
    distribution_params_ptr_ = elbo_.GetDistributionParametersPtr();

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
        // Cast into ELBO type object
        ELBO *elbo_obj_ptr = static_cast<ELBO *>(my_func_data);

        elbo_obj_ptr->ComputePredictionNormalizationConstant();

        return joint_density_fcn(x[0], elbo_obj_ptr->GetDistributionParametersPtr().get());
    };

    // Define lambda function for the ELBO objective function
    auto nlopt_wrapped_ELBO_fcn = [](const std::vector<double> &x,
                                     std::vector<double> &grad,
                                     void *my_func_data) -> double
    {
        // Cast into ELBO type object
        ELBO *elbo_obj_ptr = static_cast<ELBO *>(my_func_data);

        elbo_obj_ptr->GetDistributionParametersPtr()->SurrogateVariance = x[0]; // update the candidate variance value

        elbo_obj_ptr->ComputeELBO();

        return elbo_obj_ptr->GetELBOResult();
    };

    // Define the objective functions
    MAP_optimizer_.set_max_objective(nlopt_wrapped_MAP_fcn, &elbo_);
    MAP_optimizer_.set_lower_bounds(0.5);
    MAP_optimizer_.set_upper_bounds(1.0);
    MAP_optimizer_.set_xtol_abs(1e-9);

    ELBO_optimizer_.set_max_objective(nlopt_wrapped_ELBO_fcn, &elbo_);
    ELBO_optimizer_.set_lower_bounds(1e-3);     // smallest variance value (any smaller the ELBO evaluation will encounter nan/inf values)
    ELBO_optimizer_.set_upper_bounds(HUGE_VAL); // HUGE_VAL is basically infinity
    ELBO_optimizer_.set_xtol_abs(1e-9);

    // Extract the prediction model parameters
    model_a_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_model_A"].c_str());
    variance_r_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_variance_R"].c_str());
}

void DynamicDegradationFilterCharlie::Predict()
{
    distribution_params_ptr_->PredictionMean = model_a_ * MAP_outcome_.first[0];                               // a*x
    distribution_params_ptr_->PredictionVariance = model_a_ * model_a_ * ELBO_outcome_.first[0] + variance_r_; // a^2 * sigma^2 + r
}

void DynamicDegradationFilterCharlie::Update()
{
    // Update the binomial likelihood values
    distribution_params_ptr_->NumObservations = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations;
    distribution_params_ptr_->NumBlackTilesSeen = collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen;
    distribution_params_ptr_->FillRatio = collective_perception_algo_ptr_->GetInformedVals().X;

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
    distribution_params_ptr_->SurrogateMean = MAP_outcome_.first[0];

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
    // Execute prediction step
    Predict();

    // Execute update step
    Update();

    // Update assumed sensor accuracy
    params_ptr_->AssumedSensorAcc["b"] = MAP_outcome_.first[0];
    params_ptr_->AssumedSensorAcc["w"] = MAP_outcome_.first[0];
}