#include "algorithms/ELBO.hpp"
#include <iostream> // for debugging

/**
 * @brief PDF of a truncated normal distribution used in the surrogate model
 *
 * @param x
 * @param params
 * @return
 */
double surrogate_trunc_normal_pdf(double x, void *params)
{
    DistributionParameters *dist_params = static_cast<DistributionParameters *>(params);

    // Compute the normalization constant
    double std_dev = std::sqrt(dist_params->SurrogateVariance);

    double norm_const = std_dev * (gsl_cdf_ugaussian_P((dist_params->SurrogateUpperBound - dist_params->SurrogateMean) / std_dev) - gsl_cdf_ugaussian_P((dist_params->SurrogateLowerBound - dist_params->SurrogateMean) / std_dev));

    return gsl_ran_ugaussian_pdf((x - dist_params->SurrogateMean) / std::sqrt(dist_params->SurrogateVariance)) / norm_const;
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
    DistributionParameters *dist_params = static_cast<DistributionParameters *>(params);

    return gsl_ran_ugaussian_pdf((x - dist_params->PredictionMean) / std::sqrt(dist_params->PredictionVariance)) / dist_params->PredictionNormConst;
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
    DistributionParameters *dist_params = static_cast<DistributionParameters *>(params);

    // Compute success probability (black tile observation probability)
    double success_probability = x * dist_params->FillRatio + (1 - x) * (1 - dist_params->FillRatio);

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
    return observation_likelihood_binomial_pdf(x, params) * prediction_trunc_normal_pdf(x, params);
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

ELBO::ELBO(double a, double b, size_t n /* 1000 */)
    : workspace_size_(n),
      integration_parameters_ptr_(std::make_shared<DistributionParameters>()),
      integration_limits_(a, b)
{
    workspace_ = gsl_integration_workspace_alloc(workspace_size_);
    integrand_.function = &ELBO_integrand;
    integrand_.params = integration_parameters_ptr_.get(); // pass raw pointer to `params`
}

ELBO::~ELBO()
{
    gsl_integration_workspace_free(workspace_);
}

void ELBO::ComputePredictionNormalizationConstant()
{
    // Compute and store the prediction normalization constant
    double std_dev = std::sqrt(integration_parameters_ptr_->PredictionVariance);

    integration_parameters_ptr_->PredictionNormConst = std_dev * (gsl_cdf_ugaussian_P((integration_parameters_ptr_->PredictionUpperBound - integration_parameters_ptr_->PredictionMean) / std_dev) - gsl_cdf_ugaussian_P((integration_parameters_ptr_->PredictionLowerBound - integration_parameters_ptr_->PredictionMean) / std_dev));
}

void ELBO::ComputeELBO()
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