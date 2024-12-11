#include "algorithms/ELBO.hpp"

/**
 * @brief PDF of a truncated normal distribution used in the surrogate model
 *
 * @param x
 * @param params
 * @return
 */
double surrogate_trunc_normal_pdf(double x, void *params)
{
    SensorAccuracyDistributionParameters *dist_params = static_cast<SensorAccuracyDistributionParameters *>(params);

    // Compute the normalization constant
    double norm_const =
        dist_params->SurrogateScale * (gsl_cdf_ugaussian_P((dist_params->SurrogateUpperBound - dist_params->SurrogateLoc) / dist_params->SurrogateScale) -
                                       gsl_cdf_ugaussian_P((dist_params->SurrogateLowerBound - dist_params->SurrogateLoc) / dist_params->SurrogateScale));

    return gsl_ran_ugaussian_pdf((x - dist_params->SurrogateLoc) / dist_params->SurrogateScale) / norm_const;
}

/**
 * @brief PDF of a truncated normal distribution used in the prediction model
 *
 * @param x
 * @param params
 * @return
 */
double prediction_trunc_normal_pdf(double x, void *params)
{
    SensorAccuracyDistributionParameters *dist_params = static_cast<SensorAccuracyDistributionParameters *>(params);

    return gsl_ran_ugaussian_pdf((x - dist_params->PredictionMean) / dist_params->PredictionStdDev) / dist_params->PredictionNormConst;
}

/**
 * @brief PDF of a binomial distribution with success probability parameterized by x
 *
 * @param x Probability of correctly reporting the color of the tile
 * @param params
 * @return
 */
double observation_likelihood_binomial_pdf(double x, void *params)
{
    SensorAccuracyDistributionParameters *dist_params = static_cast<SensorAccuracyDistributionParameters *>(params);

    // Compute success probability (black tile observation probability)
    double success_probability = x / dist_params->SensorAccuracyInternalFactor * dist_params->FillRatio + (1 - x / dist_params->SensorAccuracyInternalFactor) * (1 - dist_params->FillRatio);

    return gsl_ran_binomial_pdf(dist_params->NumBlackTilesSeen, success_probability, dist_params->NumObservations);
}

/**
 * @brief PDF of the joint distribution (unnormalized)
 *
 * @param x
 * @param params
 * @return
 */
double joint_density_fcn(double x, void *params)
{
    double joint_value = observation_likelihood_binomial_pdf(x, params) * prediction_trunc_normal_pdf(x, params);
    return joint_value <= 0.0 ? 1e-9 : joint_value; // ensure that a non-zero value is returned
}

/**
 * @brief
 *
 * @param x
 * @param params
 * @return
 */
double ELBO_integrand(double x, void *params)
{
    return surrogate_trunc_normal_pdf(x, params) * std::log(joint_density_fcn(x, params) / surrogate_trunc_normal_pdf(x, params));
}

SensorAccuracyELBO::SensorAccuracyELBO(double a, double b, double internal_units_factor /* 1.0 */, size_t n /* 1000 */)
    : workspace_size_(n),
      integration_parameters_ptr_(std::make_shared<SensorAccuracyDistributionParameters>()),
      integration_limits_(a * internal_units_factor, b * internal_units_factor)
{
    workspace_ = gsl_integration_workspace_alloc(workspace_size_);
    integrand_.function = &ELBO_integrand;
    integrand_.params = integration_parameters_ptr_.get(); // pass raw pointer to `params`

    // Update the internal units factor (only set through the initializer of the ELBO)
    integration_parameters_ptr_->SensorAccuracyInternalFactor = internal_units_factor;
}

SensorAccuracyELBO::~SensorAccuracyELBO()
{
    gsl_integration_workspace_free(workspace_);
}

void SensorAccuracyELBO::Reset()
{
    integration_parameters_ptr_->Reset();
}

void SensorAccuracyELBO::ComputePredictionNormalizationConstant()
{
    // Compute and store the prediction normalization constant
    integration_parameters_ptr_->PredictionNormConst =
        integration_parameters_ptr_->PredictionStdDev * (gsl_cdf_ugaussian_P((integration_parameters_ptr_->PredictionUpperBound - integration_parameters_ptr_->PredictionMean) / integration_parameters_ptr_->PredictionStdDev) -
                                                         gsl_cdf_ugaussian_P((integration_parameters_ptr_->PredictionLowerBound - integration_parameters_ptr_->PredictionMean) / integration_parameters_ptr_->PredictionStdDev));
}

void SensorAccuracyELBO::ComputeELBO()
{
    // Compute normalization constant for the prediction model
    ComputePredictionNormalizationConstant();

    // Integrate ELBO
    gsl_integration_qags(&integrand_,
                         integration_limits_.first,
                         integration_limits_.second,
                         0,
                         1e-7,
                         workspace_size_,
                         workspace_,
                         &integration_outputs_.first,
                         &integration_outputs_.second);
}