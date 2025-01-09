#include "algorithms/DynamicDegradationFilterCharlie.hpp"

DynamicDegradationFilterCharlie::DynamicDegradationFilterCharlie(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                                                                 double lower_bound /* 0.5 + ZERO_APPROX */,
                                                                 double upper_bound /* 1.0 - ZERO_APPROX */)
    : ExtendedKalmanFilter(),
      MAP_optimizer_(nlopt::LN_NELDERMEAD, 1),
      ELBO_optimizer_(nlopt::LN_NELDERMEAD, 1),
      elbo_(lower_bound, upper_bound, SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_CHARLIE)
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
    MAP_optimizer_.set_xtol_abs(1e-12);

    ELBO_optimizer_.set_max_objective(nlopt_wrapped_ELBO_fcn, &elbo_);
    ELBO_optimizer_.set_lower_bounds(1e-4 * distribution_params_ptr_->SensorAccuracyInternalFactor); // smallest std dev value (any smaller the ELBO evaluation will encounter nan/inf values)
    ELBO_optimizer_.set_upper_bounds(HUGE_VAL);                                                      // HUGE_VAL is basically infinity
    ELBO_optimizer_.set_xtol_abs(1e-12);

    // Initialize the prior (initial guess)
    double init_mean, init_std_dev;

    init_mean = std::stod(params_ptr_->FilterSpecificParams["init_mean_MAP"]) * distribution_params_ptr_->SensorAccuracyInternalFactor;
    init_std_dev = std::stod(params_ptr_->FilterSpecificParams["init_std_dev_ELBO"]) * distribution_params_ptr_->SensorAccuracyInternalFactor;

    initial_guess_ = {init_mean, std::pow(init_std_dev, 2)};

    // Set the maximum number of past informed estimates to collect
    collective_perception_algo_ptr_->GetParamsPtr()->MaxInformedEstimateHistoryLength = collective_perception_algo_ptr_->GetParamsPtr()->MaxObservationQueueSize;

    // Populate the state prediction variables (the base class uses mean and variance, instead of mean and std dev)
    linearized_state_prediction_a_ = 1.0;
    linearized_state_prediction_b_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_model_B"].c_str()) * distribution_params_ptr_->SensorAccuracyInternalFactor;
    state_prediction_r_ = std::stod(params_ptr_->FilterSpecificParams["pred_deg_var_R"].c_str()) * std::pow(distribution_params_ptr_->SensorAccuracyInternalFactor, 2);
    update_ = initial_guess_;

    // Populate the nonlinear prediction function of the EKF (it's actually linear)
    nonlinear_state_prediction_function_ = [this](double prev_mean, const std::vector<double> &input_coefficients)
    {
        return prev_mean +
               this->params_ptr_->FilterActivationPeriodTicks * this->linearized_state_prediction_b_;
    };

    /* not using the EKF update model so not populating measurement update variables */
}

void DynamicDegradationFilterCharlie::Reset()
{
    // Call parent Reset
    ExtendedKalmanFilter::Reset();

    // Clear buffer for informed estimate (if used)
    collective_perception_algo_ptr_->GetParamsPtr()->InformedEstimateHistory.clear();

    // Reset the ELBO object and reinitialize the bounds for the internal distributions
    elbo_.Reset();

    std::pair<double, double> limits = elbo_.GetIntegrationLimits();
    distribution_params_ptr_->PredictionLowerBound = limits.first;
    distribution_params_ptr_->PredictionUpperBound = limits.second;
    distribution_params_ptr_->SurrogateLowerBound = limits.first;
    distribution_params_ptr_->SurrogateUpperBound = limits.second;
}

void DynamicDegradationFilterCharlie::Predict()
{

    // Compute the predicted values assuming normality
    ExtendedKalmanFilter::Predict(empty_vec_double_);

    distribution_params_ptr_->PredictionMean = prediction_.first;
    distribution_params_ptr_->PredictionStdDev = std::sqrt(prediction_.second);

    // Truncate the predicted mean if outside bounds
    if (distribution_params_ptr_->PredictionMean > distribution_params_ptr_->PredictionUpperBound)
    {
        distribution_params_ptr_->PredictionMean = distribution_params_ptr_->PredictionUpperBound;
    }
    else if (distribution_params_ptr_->PredictionMean < distribution_params_ptr_->PredictionLowerBound)
    {
        distribution_params_ptr_->PredictionMean = distribution_params_ptr_->PredictionLowerBound;
    }
}

void DynamicDegradationFilterCharlie::Update()
{
    // Update the binomial likelihood values
    distribution_params_ptr_->NumObservations = collective_perception_algo_ptr_->GetParamsPtr()->NumObservations;
    distribution_params_ptr_->NumBlackTilesSeen = collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen;

    // Estimate the truncated normal mean
    MAP_outcome_.first[0] = distribution_params_ptr_->PredictionMean; // use the predicted mean as the initial guess
    MAP_optimization_status_ = MAP_optimizer_.optimize(MAP_outcome_.first, MAP_outcome_.second);

    // Check if optimization is successful
    if (MAP_optimization_status_ < 0)
    {
        std::cout << "MAP Optimization unsuccessful: flag=" << MAP_optimization_status_
                  << " with f(" << MAP_outcome_.first[0] << ") = " << MAP_outcome_.second
                  << std::endl;
    }

    // Estimate the truncated normal standard deviation
    distribution_params_ptr_->SurrogateLoc = MAP_outcome_.first[0];

    ELBO_outcome_.first[0] = distribution_params_ptr_->PredictionStdDev; // use predicted std dev as initial guess
    ELBO_optimization_status_ = ELBO_optimizer_.optimize(ELBO_outcome_.first, ELBO_outcome_.second);

    // Check if optimization is successful
    if (ELBO_optimization_status_ < 0)
    {
        std::cout << "ELBO Optimization unsuccessful: flag=" << ELBO_optimization_status_
                  << " with f(" << ELBO_outcome_.first[0] << ") = " << ELBO_outcome_.second
                  << std::endl;
    }

    // Populate updated estimates (since we're not using the base Update() function)
    update_.first = MAP_outcome_.first[0];
    update_.second = std::pow(ELBO_outcome_.first[0], 2);
}

void DynamicDegradationFilterCharlie::Estimate()
{
    // Collect informed estimate into weighted average queue (if an observation queue is used)
    if (collective_perception_algo_ptr_->GetParamsPtr()->MaxInformedEstimateHistoryLength > 1)
    {
        distribution_params_ptr_->FillRatio = collective_perception_algo_ptr_->GetParamsPtr()->WeightedAverageInformedEstimate;
    }
    else
    {
        distribution_params_ptr_->FillRatio = collective_perception_algo_ptr_->GetInformedVals().X;
    }

    // Execute prediction step
    Predict();

    // Execute update step
    Update();

    // Apply exponential smoothing
    if (prev_assumed_acc_ == -1.0)
    {
        prev_assumed_acc_ = update_.first / this->distribution_params_ptr_->SensorAccuracyInternalFactor;
    }
    else
    {
        params_ptr_->AssumedSensorAcc["b"] = exponential_smoothing_factor_ * update_.first / this->distribution_params_ptr_->SensorAccuracyInternalFactor + (1 - exponential_smoothing_factor_) * prev_assumed_acc_;
        prev_assumed_acc_ = params_ptr_->AssumedSensorAcc["b"];
    }

    params_ptr_->AssumedSensorAcc["w"] = params_ptr_->AssumedSensorAcc["b"]; // black tile accuracy is equal to white tile accuracy
}